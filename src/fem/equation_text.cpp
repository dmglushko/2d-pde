#include "pde2d/fem/equation_text.hpp"

#include "derivative.hpp"
#include "symengine_expression.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::fem {

namespace {

using SymEngine::Basic;
using SymEngine::RCP;

// --- the line as tokens
// ---------------------------------------------------------

enum class TokenKind {
  number,
  name,
  plus,
  minus,
  times,
  divide,
  power,
  open,
  close,
  open_bracket,
  close_bracket,
  comma,
  equals,
  dot,
  nabla,
  delta,
  squared,
  other,
  end
};

struct Token {
  TokenKind kind;
  std::size_t begin;
  std::size_t end;
};

struct Symbol {
  std::string_view spelling;
  TokenKind kind;
};

constexpr std::string_view no_break_space = "\u00a0";

// The non-ASCII symbols, in UTF-8, and the two-character ASCII one.
constexpr std::array<Symbol, 8> symbols{{
    {"**", TokenKind::power},
    {"∇", TokenKind::nabla},   // ∇
    {"·", TokenKind::dot},     // ·
    {"⋅", TokenKind::dot},     // ⋅
    {"Δ", TokenKind::delta},   // Δ (Greek capital delta)
    {"∆", TokenKind::delta},   // ∆ (increment)
    {"²", TokenKind::squared}, // ²
    {" ", TokenKind::end},     // no-break space: skipped
}};

bool is_name_start(char c) {
  return std::isalpha(static_cast<unsigned char>(c)) != 0 || c == '_';
}

bool is_name_char(char c) {
  return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

bool is_digit(char c) { return std::isdigit(static_cast<unsigned char>(c)); }

std::vector<Token> tokenize(std::string_view text) {
  std::vector<Token> tokens;
  std::size_t i = 0;
  while (i < text.size()) {
    const char c = text[i];
    if (std::isspace(static_cast<unsigned char>(c)) != 0) {
      ++i;
      continue;
    }
    const auto start = i;
    bool matched = false;
    for (const auto &symbol : symbols)
      if (text.substr(i).starts_with(symbol.spelling)) {
        i += symbol.spelling.size();
        if (symbol.kind != TokenKind::end)
          tokens.push_back({symbol.kind, start, i});
        matched = true;
        break;
      }
    if (matched)
      continue;
    if (is_name_start(c)) {
      while (i < text.size() && is_name_char(text[i]))
        ++i;
      tokens.push_back({TokenKind::name, start, i});
      continue;
    }
    if (is_digit(c) ||
        (c == '.' && i + 1 < text.size() && is_digit(text[i + 1]))) {
      while (i < text.size() && is_digit(text[i]))
        ++i;
      if (i < text.size() && text[i] == '.') {
        ++i;
        while (i < text.size() && is_digit(text[i]))
          ++i;
      }
      if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
        auto j = i + 1;
        if (j < text.size() && (text[j] == '+' || text[j] == '-'))
          ++j;
        if (j < text.size() && is_digit(text[j])) {
          i = j;
          while (i < text.size() && is_digit(text[i]))
            ++i;
        }
      }
      tokens.push_back({TokenKind::number, start, i});
      continue;
    }
    TokenKind kind = TokenKind::other;
    switch (c) {
    case '+':
      kind = TokenKind::plus;
      break;
    case '-':
      kind = TokenKind::minus;
      break;
    case '*':
      kind = TokenKind::times;
      break;
    case '/':
      kind = TokenKind::divide;
      break;
    case '^':
      kind = TokenKind::power;
      break;
    case '(':
      kind = TokenKind::open;
      break;
    case ')':
      kind = TokenKind::close;
      break;
    case '[':
      kind = TokenKind::open_bracket;
      break;
    case ']':
      kind = TokenKind::close_bracket;
      break;
    case ',':
      kind = TokenKind::comma;
      break;
    case '=':
      kind = TokenKind::equals;
      break;
    default:
      break;
    }
    ++i;
    // One character, also a whole UTF-8 sequence.
    if (kind == TokenKind::other)
      while (i < text.size() &&
             (static_cast<unsigned char>(text[i]) & 0xC0) == 0x80)
        ++i;
    tokens.push_back({kind, start, i});
  }
  tokens.push_back({TokenKind::end, text.size(), text.size()});
  return tokens;
}

// --- the line as a tree
// -----------------------------------------------------------

// The derivatives of u a term may hold; `none` is a term without u.
enum class Term { none, u, u_x, u_y, u_xx, u_xy, u_yy, u_t, u_tt };

enum class Mode { equation, condition };

enum class NodeKind {
  // Plain text Expression reads as it is (a number, a name, a function
  // call, or +, -, *, /, ^ and parentheses of such).
  expression,
  unknown, // u or one of its derivatives (`term`)
  normal,
  tangent,
  normal_derivative,
  tangential_derivative,
  negate,
  add,
  subtract,
  multiply, // *, · or a factor in front of ∇ or Δ
  divide,
  power,
  gradient,
  divergence,
  laplacian,
  dot,
  tuple // a vector or a matrix (two rows)
};

struct Node {
  NodeKind kind = NodeKind::expression;
  std::size_t begin = 0;
  std::size_t end = 0;
  std::vector<Node> children{};
  Term term = Term::none;
  // The node's text is plain Expression text without u: it is read by
  // Expression as a whole.
  bool plain = false;
  // multiply only: written with · (the dot product of two vectors).
  bool dot_symbol = false;
  // multiply only: a factor written in front of ∇ or Δ (k∇u).
  bool juxtaposed = false;
};

// u_x, u_yx, u_tt...: the derivative a name means, if it names one of u.
// Throws through `fail` for derivatives the forms do not have.
struct UnknownName {
  bool is_unknown = false;
  Term term = Term::none;
  std::string problem{}; // why it cannot be used, if it cannot
};

UnknownName unknown_name(std::string_view name, Mode mode) {
  if (name == "u")
    return {true, Term::u};
  if (!name.starts_with("u_") || name.size() < 3)
    return {};
  const auto letters = name.substr(2);
  if (letters == "n" || letters == "tau") {
    if (mode == Mode::condition)
      return {true, Term::none};
    return {true, Term::none,
            "is a derivative along the boundary's normal or tangent: it "
            "belongs in a boundary condition"};
  }
  for (const char c : letters)
    if (c != 'x' && c != 'y' && c != 't')
      return {};
  const auto count = [&](char c) {
    return static_cast<int>(std::ranges::count(letters, c));
  };
  const int nx = count('x'), ny = count('y'), nt = count('t');
  if (nt > 0 && nx + ny > 0)
    return {true, Term::none,
            "is a mixed derivative in space and time, which the "
            "equation's form does not have"};
  if (nx + ny + nt > 2)
    return {true, Term::none,
            "is a third derivative; the equation has at most second "
            "derivatives"};
  if (nt == 1)
    return {true, Term::u_t};
  if (nt == 2)
    return {true, Term::u_tt};
  if (nx == 1 && ny == 0)
    return {true, Term::u_x};
  if (nx == 0 && ny == 1)
    return {true, Term::u_y};
  if (nx == 2)
    return {true, Term::u_xx};
  if (ny == 2)
    return {true, Term::u_yy};
  return {true, Term::u_xy};
}

class Parser {
public:
  Parser(std::string_view text, Mode mode, std::string prefix)
      : text_(text), mode_(mode), prefix_(std::move(prefix)),
        tokens_(tokenize(text)) {}

  // left = right.
  std::pair<Node, Node> line() {
    // Only the '=' outside parentheses: Piecewise conditions have their
    // own (x == 0, x <= 1).
    std::size_t equals = 0;
    std::vector<std::size_t> open;
    for (const auto &token : tokens_) {
      if (token.kind == TokenKind::open ||
          token.kind == TokenKind::open_bracket)
        open.push_back(token.begin);
      else if (token.kind == TokenKind::close ||
               token.kind == TokenKind::close_bracket) {
        if (open.empty())
          fail("it has a '" + std::string(spelling(token)) + "' at character " +
               std::to_string(position(token.begin)) + " that closes nothing");
        open.pop_back();
      } else if (token.kind == TokenKind::equals && open.empty())
        ++equals;
    }
    if (!open.empty())
      fail("it has a '" + std::string(text_.substr(open.back(), 1)) +
           "' at character " + std::to_string(position(open.back())) +
           " that is not closed");
    if (equals == 0)
      fail("it has no '=': write it as left side = right side, for example " +
           std::string(mode_ == Mode::equation ? "-lap(u) = 1" : "u = 0"));
    if (equals > 1)
      fail("it has more than one '='");
    auto left = expression();
    expect(TokenKind::equals, "'='");
    auto right = expression();
    expect(TokenKind::end, "the end of the line");
    return {std::move(left), std::move(right)};
  }

  [[noreturn]] void fail(const std::string &reason) const {
    throw std::invalid_argument(prefix_ + reason);
  }

  std::string quote(const Node &node) const {
    return "'" + std::string(text_.substr(node.begin, node.end - node.begin)) +
           "'";
  }

  std::string_view text_of(const Node &node) const {
    return text_.substr(node.begin, node.end - node.begin);
  }

  Mode mode() const { return mode_; }
  const std::string &prefix() const { return prefix_; }

private:
  std::string_view text_;
  Mode mode_;
  std::string prefix_;
  std::vector<Token> tokens_;
  std::size_t at_ = 0;

  const Token &peek(std::size_t ahead = 0) const {
    return tokens_[std::min(at_ + ahead, tokens_.size() - 1)];
  }
  const Token &take() { return tokens_[std::min(at_++, tokens_.size() - 1)]; }
  bool is(TokenKind kind) const { return peek().kind == kind; }

  std::string_view spelling(const Token &token) const {
    return text_.substr(token.begin, token.end - token.begin);
  }

  // The character position (from 1, counting UTF-8 sequences as one).
  std::size_t position(std::size_t byte) const {
    std::size_t count = 0;
    for (std::size_t i = 0; i < byte && i < text_.size(); ++i)
      if ((static_cast<unsigned char>(text_[i]) & 0xC0) != 0x80)
        ++count;
    return count + 1;
  }

  [[noreturn]] void unexpected(const std::string &wanted) const {
    const auto &token = peek();
    if (token.kind == TokenKind::end)
      fail("it ends where " + wanted + " is expected");
    std::string what = "'" + std::string(spelling(token)) + "'";
    std::string hint;
    if (token.kind == TokenKind::name || token.kind == TokenKind::number ||
        token.kind == TokenKind::open)
      hint = " (a product needs '*', as in 2*u)";
    fail("it has " + what + " at character " +
         std::to_string(position(token.begin)) + " where " + wanted +
         " is expected" + hint);
  }

  void expect(TokenKind kind, const std::string &wanted) {
    if (!is(kind))
      unexpected(wanted);
    take();
  }

  static Node make(NodeKind kind, std::size_t begin, std::size_t end,
                   std::vector<Node> children = {}) {
    Node node;
    node.kind = kind;
    node.begin = begin;
    node.end = end;
    node.children = std::move(children);
    return node;
  }

  static bool all_plain(const std::vector<Node> &nodes) {
    return std::ranges::all_of(nodes, [](const Node &n) { return n.plain; });
  }

  // Precedence, loosest first: + and -; the dot ·; * and /; a sign; a
  // factor written in front of ∇ or Δ (k∇u); ^; then ∇ and Δ, whose
  // operand is a power or such a product (Δx^2 is Δ(x^2), ∇·k∇u is
  // ∇·(k∇u)). So n·k∇u is n·(k∇u), and 1/2 Δu is 1/(2Δu), refused.
  Node expression() {
    auto left = dotted();
    while (is(TokenKind::plus) || is(TokenKind::minus)) {
      const bool plus = take().kind == TokenKind::plus;
      auto right = dotted();
      const auto begin = left.begin, end = right.end;
      std::vector<Node> children;
      children.push_back(std::move(left));
      children.push_back(std::move(right));
      left = make(plus ? NodeKind::add : NodeKind::subtract, begin, end,
                  std::move(children));
      left.plain = all_plain(left.children);
    }
    return left;
  }

  // a·b·c: the dot product of two vectors, the product otherwise.
  Node dotted() {
    auto left = term();
    while (is(TokenKind::dot)) {
      take();
      auto right = term();
      const auto begin = left.begin, end = right.end;
      std::vector<Node> children;
      children.push_back(std::move(left));
      children.push_back(std::move(right));
      left = make(NodeKind::multiply, begin, end, std::move(children));
      left.dot_symbol = true;
    }
    return left;
  }

  Node term() {
    auto left = unary();
    while (is(TokenKind::times) || is(TokenKind::divide)) {
      const auto kind = take().kind == TokenKind::times ? NodeKind::multiply
                                                        : NodeKind::divide;
      auto right = unary();
      const auto begin = left.begin, end = right.end;
      std::vector<Node> children;
      children.push_back(std::move(left));
      children.push_back(std::move(right));
      left = make(kind, begin, end, std::move(children));
      left.plain = all_plain(left.children);
    }
    return left;
  }

  // k∇u, A ∇u, 2Δu: a factor directly in front of ∇ or Δ multiplies it.
  Node juxtaposed() {
    auto left = power();
    while (is(TokenKind::nabla) || is(TokenKind::delta)) {
      auto right = power();
      const auto begin = left.begin, end = right.end;
      std::vector<Node> children;
      children.push_back(std::move(left));
      children.push_back(std::move(right));
      left = make(NodeKind::multiply, begin, end, std::move(children));
      left.juxtaposed = true;
    }
    return left;
  }

  Node unary() {
    if (is(TokenKind::minus) || is(TokenKind::plus)) {
      const auto &sign = take();
      auto operand = unary();
      if (sign.kind == TokenKind::plus) {
        operand.begin = sign.begin;
        return operand;
      }
      const auto end = operand.end;
      std::vector<Node> children;
      children.push_back(std::move(operand));
      auto node = make(NodeKind::negate, sign.begin, end, std::move(children));
      node.plain = all_plain(node.children);
      return node;
    }
    return juxtaposed();
  }

  Node power() {
    auto base = atom();
    if (!is(TokenKind::power))
      return base;
    take();
    auto exponent = unary();
    const auto begin = base.begin, end = exponent.end;
    std::vector<Node> children;
    children.push_back(std::move(base));
    children.push_back(std::move(exponent));
    auto node = make(NodeKind::power, begin, end, std::move(children));
    node.plain = all_plain(node.children);
    return node;
  }

  // The operand of ∇, ∇·, ∇² and Δ: a power, or a product with a factor
  // in front of ∇ or Δ (another of them is an atom).
  Node nabla_operand() { return juxtaposed(); }

  Node nabla() {
    const auto &symbol = take();
    NodeKind kind = NodeKind::laplacian;
    if (symbol.kind == TokenKind::nabla) {
      kind = NodeKind::gradient;
      if (is(TokenKind::dot)) {
        take();
        kind = NodeKind::divergence;
      } else if (is(TokenKind::squared)) {
        take();
        kind = NodeKind::laplacian;
      }
    }
    auto operand = nabla_operand();
    const auto end = operand.end;
    std::vector<Node> children;
    children.push_back(std::move(operand));
    return make(kind, symbol.begin, end, std::move(children));
  }

  // The tokens of a function's arguments, from '(' to its ')': the
  // arguments are Expression's text, read as they are (Piecewise with its
  // conditions included). Returns the index after ')'.
  std::size_t skip_arguments(std::size_t open) const {
    int depth = 0;
    for (std::size_t i = open; i < tokens_.size(); ++i) {
      const auto kind = tokens_[i].kind;
      if (kind == TokenKind::open || kind == TokenKind::open_bracket)
        ++depth;
      else if (kind == TokenKind::close || kind == TokenKind::close_bracket) {
        if (--depth == 0)
          return i + 1;
      } else if (kind == TokenKind::end)
        break;
    }
    fail("it has a '(' at character " +
         std::to_string(position(tokens_[open].begin)) + " that is not closed");
  }

  // Whether the tokens [first, last) mention u, the normal or an
  // operator of the line (they then cannot be Expression's text).
  std::optional<std::string> line_part(std::size_t first,
                                       std::size_t last) const {
    for (std::size_t i = first; i < last; ++i) {
      const auto &token = tokens_[i];
      switch (token.kind) {
      case TokenKind::nabla:
      case TokenKind::delta:
      case TokenKind::dot:
      case TokenKind::squared:
        return std::string(spelling(token));
      case TokenKind::name: {
        const auto name = spelling(token);
        if (unknown_name(name, mode_).is_unknown)
          return std::string(name);
        if (mode_ == Mode::condition && (name == "n" || name == "tau"))
          return std::string(name);
        if ((name == "grad" || name == "div" || name == "lap" ||
             name == "dot") &&
            i + 1 < last && tokens_[i + 1].kind == TokenKind::open)
          return std::string(name);
        break;
      }
      default:
        break;
      }
    }
    return std::nullopt;
  }

  Node atom() {
    const auto &token = peek();
    switch (token.kind) {
    case TokenKind::number: {
      take();
      auto node = make(NodeKind::expression, token.begin, token.end);
      node.plain = true;
      return node;
    }
    case TokenKind::name:
      return named();
    case TokenKind::open:
    case TokenKind::open_bracket:
      return group();
    case TokenKind::nabla:
    case TokenKind::delta:
      return nabla();
    default:
      unexpected("a number, a name, u or '('");
    }
  }

  Node named() {
    const auto &token = take();
    const auto name = spelling(token);
    if (is(TokenKind::open)) {
      if (name == "grad" || name == "div" || name == "lap" || name == "dot")
        return operator_call(token, name);
      const auto open = at_;
      const auto after = skip_arguments(open);
      if (const auto part = line_part(open, after))
        fail("it applies '" + std::string(name) + "' to '" + *part +
             "': a function may take only coefficients (x, y, t and "
             "parameters), since the line must be linear in u");
      at_ = after;
      auto node =
          make(NodeKind::expression, token.begin, tokens_[after - 1].end);
      node.plain = true;
      return node;
    }
    if (const auto unknown = unknown_name(name, mode_); unknown.is_unknown) {
      if (!unknown.problem.empty())
        fail("'" + std::string(name) + "' " + unknown.problem);
      auto node = make(NodeKind::unknown, token.begin, token.end);
      node.term = unknown.term;
      if (name == "u_n")
        node.kind = NodeKind::normal_derivative;
      else if (name == "u_tau")
        node.kind = NodeKind::tangential_derivative;
      return node;
    }
    if (mode_ == Mode::condition && name == "n")
      return make(NodeKind::normal, token.begin, token.end);
    if (mode_ == Mode::condition && name == "tau")
      return make(NodeKind::tangent, token.begin, token.end);
    auto node = make(NodeKind::expression, token.begin, token.end);
    node.plain = true;
    return node;
  }

  Node operator_call(const Token &token, std::string_view name) {
    take(); // (
    std::vector<Node> arguments;
    arguments.push_back(expression());
    while (is(TokenKind::comma)) {
      take();
      arguments.push_back(expression());
    }
    const auto &close = peek();
    expect(TokenKind::close, "')'");
    const std::size_t wanted = name == "dot" ? 2 : 1;
    if (arguments.size() != wanted)
      fail("'" + std::string(name) + "' takes " +
           (wanted == 1 ? std::string("one argument")
                        : std::string("two arguments")) +
           ", not " + std::to_string(arguments.size()));
    const auto kind = name == "grad"  ? NodeKind::gradient
                      : name == "div" ? NodeKind::divergence
                      : name == "lap" ? NodeKind::laplacian
                                      : NodeKind::dot;
    return make(kind, token.begin, close.end, std::move(arguments));
  }

  // ( expression ), or a vector or matrix: (a, b), [a, b], [[...], [...]].
  Node group() {
    const auto &open = take();
    const bool bracket = open.kind == TokenKind::open_bracket;
    const auto closing = bracket ? TokenKind::close_bracket : TokenKind::close;
    std::vector<Node> items;
    items.push_back(expression());
    while (is(TokenKind::comma)) {
      take();
      items.push_back(expression());
    }
    const auto &close = peek();
    expect(closing, bracket ? "']'" : "')'");
    if (items.size() == 1 && !bracket) {
      auto inner = std::move(items.front());
      // The parentheses belong to the node's text (plain text stays
      // plain).
      inner.begin = open.begin;
      inner.end = close.end;
      return inner;
    }
    auto node = make(NodeKind::tuple, open.begin, close.end, std::move(items));
    if (node.children.size() != 2)
      fail(quote(node) + " has " + std::to_string(node.children.size()) +
           " entries; vectors have two entries and matrices two rows");
    return node;
  }
};

// --- values: linear in u
// --------------------------------------------------------------

// The normal's component a term carries (only in a condition).
enum class Normal { one, n_x, n_y };

using Key = std::pair<Term, Normal>;
// A scalar that is linear in u: a coefficient per derivative of u (and
// normal component); the key {none, one} is the part without u.
using Form = std::map<Key, RCP<const Basic>>;

enum class Shape { scalar, vector, matrix };

struct Value {
  Shape shape = Shape::scalar;
  // 1, 2 (x, y) or 4 (row by row) forms.
  std::vector<Form> parts{};
};

bool zero_number(const Basic &form) {
  return SymEngine::is_a_Number(form) &&
         SymEngine::down_cast<const SymEngine::Number &>(form).is_zero();
}

void accumulate(Form &form, const Key &key, const RCP<const Basic> &value) {
  if (zero_number(*value))
    return;
  const auto it = form.find(key);
  if (it == form.end()) {
    form.emplace(key, value);
    return;
  }
  it->second = SymEngine::add(it->second, value);
  if (zero_number(*it->second))
    form.erase(it);
}

Value scalar(Form form) { return {Shape::scalar, {std::move(form)}}; }

Value constant(const RCP<const Basic> &value) {
  Form form;
  accumulate(form, {Term::none, Normal::one}, value);
  return scalar(std::move(form));
}

Value unknown(Term term, Normal normal = Normal::one) {
  Form form;
  form.emplace(Key{term, normal}, SymEngine::integer(1));
  return scalar(std::move(form));
}

const char *shape_name(Shape shape) {
  switch (shape) {
  case Shape::scalar:
    return "a number";
  case Shape::vector:
    return "a vector";
  case Shape::matrix:
    return "a matrix";
  }
  return "";
}

// True when the form holds no u and no normal.
bool is_constant(const Form &form) {
  return std::ranges::all_of(form, [](const auto &entry) {
    return entry.first == Key{Term::none, Normal::one};
  });
}

RCP<const Basic> constant_part(const Form &form) {
  const auto it = form.find({Term::none, Normal::one});
  return it == form.end() ? RCP<const Basic>(SymEngine::integer(0))
                          : it->second;
}

class Evaluator {
public:
  explicit Evaluator(const Parser &parser) : parser_(parser) {}

  Value value(const Node &node) {
    if (node.plain)
      return constant(expression(node));
    switch (node.kind) {
    case NodeKind::expression:
      return constant(expression(node));
    case NodeKind::unknown:
      return unknown(node.term);
    case NodeKind::normal:
      return {Shape::vector,
              {unknown(Term::none, Normal::n_x).parts[0],
               unknown(Term::none, Normal::n_y).parts[0]}};
    case NodeKind::tangent: {
      // tau = (-n_y, n_x).
      Form x, y;
      x.emplace(Key{Term::none, Normal::n_y}, SymEngine::integer(-1));
      y.emplace(Key{Term::none, Normal::n_x}, SymEngine::integer(1));
      return {Shape::vector, {std::move(x), std::move(y)}};
    }
    case NodeKind::normal_derivative: {
      Form form;
      form.emplace(Key{Term::u_x, Normal::n_x}, SymEngine::integer(1));
      form.emplace(Key{Term::u_y, Normal::n_y}, SymEngine::integer(1));
      return scalar(std::move(form));
    }
    case NodeKind::tangential_derivative: {
      Form form;
      form.emplace(Key{Term::u_x, Normal::n_y}, SymEngine::integer(-1));
      form.emplace(Key{Term::u_y, Normal::n_x}, SymEngine::integer(1));
      return scalar(std::move(form));
    }
    case NodeKind::negate:
      return negate(value(node.children[0]));
    case NodeKind::add:
    case NodeKind::subtract: {
      auto left = value(node.children[0]);
      auto right = value(node.children[1]);
      if (left.shape != right.shape)
        parser_.fail(parser_.quote(node) + " " +
                     (node.kind == NodeKind::add ? "adds" : "subtracts") + " " +
                     shape_name(right.shape) + " and " +
                     shape_name(left.shape));
      if (node.kind == NodeKind::subtract)
        right = negate(std::move(right));
      for (std::size_t i = 0; i < left.parts.size(); ++i)
        for (const auto &[key, coefficient] : right.parts[i])
          accumulate(left.parts[i], key, coefficient);
      return left;
    }
    case NodeKind::multiply:
      return multiply(node, value(node.children[0]), value(node.children[1]));
    case NodeKind::divide:
      return divide(node, value(node.children[0]), value(node.children[1]));
    case NodeKind::power:
      return power(node, value(node.children[0]), value(node.children[1]));
    case NodeKind::gradient:
      return gradient(node, value(node.children[0]));
    case NodeKind::divergence:
      return divergence(node, value(node.children[0]));
    case NodeKind::laplacian: {
      auto operand = value(node.children[0]);
      if (operand.shape != Shape::scalar)
        parser_.fail(parser_.quote(node) + " takes the Laplacian of " +
                     shape_name(operand.shape) +
                     "; it needs a number, as in lap(u)");
      return divergence(node, gradient(node, std::move(operand)));
    }
    case NodeKind::dot: {
      auto left = value(node.children[0]);
      auto right = value(node.children[1]);
      if (left.shape != Shape::vector || right.shape != Shape::vector)
        parser_.fail(parser_.quote(node) + " takes the dot product of " +
                     shape_name(left.shape) + " and " +
                     shape_name(right.shape) + "; it needs two vectors");
      return dot(node, left, right);
    }
    case NodeKind::tuple:
      return tuple(node);
    }
    return {};
  }

private:
  const Parser &parser_;

  // A plain part, read by Expression as it is written.
  RCP<const Basic> expression(const Node &node) {
    try {
      // A no-break space, skipped by the line, is a space to Expression.
      std::string text(parser_.text_of(node));
      for (auto at = text.find(no_break_space); at != std::string::npos;
           at = text.find(no_break_space, at + 1))
        text.replace(at, no_break_space.size(), " ");
      const Expression read(text);
      return detail::ExpressionAccess::impl(read).form;
    } catch (const std::invalid_argument &e) {
      parser_.fail(e.what());
    }
  }

  // A coefficient the line built, checked like an expression.
  Expression checked(const RCP<const Basic> &form) {
    try {
      return detail::expression_from_form(form);
    } catch (const std::invalid_argument &e) {
      parser_.fail(e.what());
    }
  }

  static Value negate(Value value) {
    for (auto &part : value.parts)
      for (auto &[key, coefficient] : part)
        coefficient = SymEngine::neg(coefficient);
    return value;
  }

  Form product(const Node &node, const Form &a, const Form &b) {
    Form result;
    for (const auto &[ka, ca] : a)
      for (const auto &[kb, cb] : b) {
        if (ka.first != Term::none && kb.first != Term::none)
          parser_.fail(parser_.quote(node) +
                       " multiplies u by u: the line must be linear in u");
        if (ka.second != Normal::one && kb.second != Normal::one)
          parser_.fail(parser_.quote(node) +
                       " multiplies the normal or tangent by itself; they "
                       "may appear only once, as in n·∇u");
        const Key key{ka.first != Term::none ? ka.first : kb.first,
                      ka.second != Normal::one ? ka.second : kb.second};
        accumulate(result, key, SymEngine::mul(ca, cb));
      }
    return result;
  }

  Form sum(Form a, const Form &b) {
    for (const auto &[key, coefficient] : b)
      accumulate(a, key, coefficient);
    return a;
  }

  Value dot(const Node &node, const Value &a, const Value &b) {
    return scalar(sum(product(node, a.parts[0], b.parts[0]),
                      product(node, a.parts[1], b.parts[1])));
  }

  Value multiply(const Node &node, Value a, Value b) {
    if (a.shape == Shape::scalar || b.shape == Shape::scalar) {
      const bool a_scalar = a.shape == Shape::scalar;
      const Form &factor = a_scalar ? a.parts[0] : b.parts[0];
      Value result = a_scalar ? std::move(b) : std::move(a);
      for (auto &part : result.parts)
        part = product(node, factor, part);
      return result;
    }
    if (a.shape == Shape::vector && b.shape == Shape::vector) {
      if (node.dot_symbol)
        return dot(node, a, b);
      parser_.fail(parser_.quote(node) +
                   " multiplies two vectors: write their dot product as "
                   "a·b or dot(a, b)");
    }
    if (a.shape == Shape::matrix && b.shape == Shape::vector) {
      Value result{Shape::vector, {}};
      for (int row = 0; row < 2; ++row)
        result.parts.push_back(
            sum(product(node, a.parts[2 * row], b.parts[0]),
                product(node, a.parts[2 * row + 1], b.parts[1])));
      return result;
    }
    if (a.shape == Shape::matrix && b.shape == Shape::matrix) {
      Value result{Shape::matrix, {}};
      for (int row = 0; row < 2; ++row)
        for (int column = 0; column < 2; ++column)
          result.parts.push_back(
              sum(product(node, a.parts[2 * row], b.parts[column]),
                  product(node, a.parts[2 * row + 1], b.parts[2 + column])));
      return result;
    }
    parser_.fail(parser_.quote(node) + " multiplies a vector by a matrix " +
                 "from the left; write the matrix first (A ∇u)");
  }

  Value divide(const Node &node, Value a, const Value &b) {
    if (b.shape != Shape::scalar || !is_constant(b.parts[0]))
      parser_.fail(parser_.quote(node) + " divides by " +
                   (b.shape != Shape::scalar ? std::string(shape_name(b.shape))
                                             : std::string("u or the normal")) +
                   "; only coefficients may divide" +
                   (node.children[1].juxtaposed
                        ? std::string(" (a factor written in front of ∇ or "
                                      "Δ belongs to it: write (1/2)Δu or "
                                      "Δu/2, not 1/2 Δu)")
                        : std::string()));
    const auto denominator = constant_part(b.parts[0]);
    if (zero_number(*denominator))
      parser_.fail(parser_.quote(node) + " divides by zero");
    for (auto &part : a.parts)
      for (auto &[key, coefficient] : part)
        coefficient = SymEngine::div(coefficient, denominator);
    return a;
  }

  Value power(const Node &node, const Value &base, const Value &exponent) {
    if (base.shape != Shape::scalar || exponent.shape != Shape::scalar ||
        !is_constant(base.parts[0]) || !is_constant(exponent.parts[0]))
      parser_.fail(parser_.quote(node) +
                   " raises u, the normal, a vector or a matrix to a power, "
                   "or uses one as an exponent: the line must be linear in u");
    // Exactly integral floating exponents become integers, as when
    // Expression reads them.
    return constant(detail::substitute_powers(
        SymEngine::pow(constant_part(base.parts[0]),
                       constant_part(exponent.parts[0])),
        {}));
  }

  static std::optional<Term> raised(Term term, detail::Variable variable) {
    const bool x = variable == detail::Variable::x;
    switch (term) {
    case Term::none:
      return Term::none;
    case Term::u:
      return x ? Term::u_x : Term::u_y;
    case Term::u_x:
      return x ? Term::u_xx : Term::u_xy;
    case Term::u_y:
      return x ? Term::u_xy : Term::u_yy;
    default:
      return std::nullopt;
    }
  }

  // d/dx or d/dy of a scalar form: the coefficients' derivatives and the
  // derivatives of u one order higher.
  Form derivative(const Node &node, const Form &form,
                  detail::Variable variable) {
    Form result;
    for (const auto &[key, coefficient] : form) {
      if (key.second != Normal::one)
        parser_.fail(parser_.quote(node) +
                     " differentiates the normal or tangent; they may "
                     "appear only outside derivatives, as in n·∇u");
      const auto term = raised(key.first, variable);
      if (!term) {
        if (key.first == Term::u_t || key.first == Term::u_tt)
          parser_.fail(parser_.quote(node) +
                       " takes a space derivative of u_t or u_tt, which "
                       "the equation's form does not have");
        parser_.fail(parser_.quote(node) +
                     " takes a third derivative of u; the equation has at "
                     "most second derivatives");
      }
      Expression written = checked(coefficient);
      if (!written.facts().zero) {
        const Expression changed = detail::derivative(written, variable);
        accumulate(result, key, detail::ExpressionAccess::impl(changed).form);
      }
      if (key.first != Term::none)
        accumulate(result, {*term, Normal::one}, coefficient);
    }
    return result;
  }

  Value gradient(const Node &node, const Value &operand) {
    if (operand.shape != Shape::scalar)
      parser_.fail(parser_.quote(node) + " takes the gradient of " +
                   shape_name(operand.shape) +
                   "; it needs a number, as in grad(u)");
    return {Shape::vector,
            {derivative(node, operand.parts[0], detail::Variable::x),
             derivative(node, operand.parts[0], detail::Variable::y)}};
  }

  Value divergence(const Node &node, const Value &operand) {
    if (operand.shape != Shape::vector)
      parser_.fail(parser_.quote(node) + " takes the divergence of " +
                   shape_name(operand.shape) +
                   "; it needs a vector, as in div(k*grad(u)) or ∇·(k∇u)");
    return scalar(sum(derivative(node, operand.parts[0], detail::Variable::x),
                      derivative(node, operand.parts[1], detail::Variable::y)));
  }

  Value tuple(const Node &node) {
    auto first = value(node.children[0]);
    auto second = value(node.children[1]);
    if (first.shape == Shape::scalar && second.shape == Shape::scalar)
      return {Shape::vector,
              {std::move(first.parts[0]), std::move(second.parts[0])}};
    if (first.shape == Shape::vector && second.shape == Shape::vector)
      return {Shape::matrix,
              {std::move(first.parts[0]), std::move(first.parts[1]),
               std::move(second.parts[0]), std::move(second.parts[1])}};
    parser_.fail(parser_.quote(node) +
                 " mixes entries of different kinds; a vector has two "
                 "numbers and a matrix two rows of two");
  }

public:
  // left - right, which must be a number.
  Form difference(const Node &left, const Node &right) {
    auto a = value(left);
    auto b = value(right);
    if (a.shape != Shape::scalar || b.shape != Shape::scalar)
      parser_.fail(std::string("it has ") +
                   shape_name(a.shape != Shape::scalar ? a.shape : b.shape) +
                   " on the " + (a.shape != Shape::scalar ? "left" : "right") +
                   " side; both sides must be numbers");
    for (const auto &[key, coefficient] : b.parts[0])
      accumulate(a.parts[0], key, SymEngine::neg(coefficient));
    return std::move(a.parts[0]);
  }

  Expression coefficient(const Form &form, const Key &key) {
    const auto it = form.find(key);
    return it == form.end() ? Expression() : checked(it->second);
  }
};

std::string quoted(std::string_view text) {
  return "'" + std::string(text) + "'";
}

bool has_u(const Form &form) {
  return std::ranges::any_of(
      form, [](const auto &entry) { return entry.first.first != Term::none; });
}

// --- printing
// ------------------------------------------------------------------------

struct Printed {
  bool negative;
  std::string magnitude; // without the sign
};

// A coefficient's form as text, split into its sign and the rest.
Printed printed(const RCP<const Basic> &form) {
  using namespace SymEngine;
  bool negative = false;
  if (is_a_Number(*form))
    negative = down_cast<const Number &>(*form).is_negative();
  else if (is_a<Mul>(*form))
    negative =
        down_cast<const Number &>(*down_cast<const Mul &>(*form).get_coef())
            .is_negative();
  const auto magnitude = negative ? neg(form) : form;
  return {negative, detail::form_text(magnitude)};
}

// Throws when a coefficient uses a parameter that the line would read as
// u, a derivative of u, the normal or the tangent: printed as it is, the
// line would mean another problem.
void check_names(
    const std::vector<std::pair<const Expression *, const char *>> &terms,
    const Expression &right, Mode mode, std::string_view what) {
  const auto check = [&](const Expression &expression, const std::string &of) {
    const auto &form = detail::ExpressionAccess::impl(expression).form;
    for (const auto &symbol : SymEngine::free_symbols(*form)) {
      const auto name =
          SymEngine::down_cast<const SymEngine::Symbol &>(*symbol).get_name();
      const bool unknown = unknown_name(name, mode).is_unknown;
      if (unknown ||
          (mode == Mode::condition && (name == "n" || name == "tau")))
        throw std::invalid_argument(
            "Cannot write the " + std::string(what) + " as one line: " + of +
            " uses the parameter '" + name + "', which a line reads as " +
            (unknown ? "u or a derivative of u" : "the normal or tangent") +
            "; rename the parameter");
    }
  };
  for (const auto &[expression, name] : terms)
    check(*expression, "the coefficient of " + std::string(name));
  check(right, "the right side");
}

std::string
line(const std::vector<std::pair<const Expression *, const char *>> &terms,
     const Expression &right) {
  std::string out;
  for (const auto &[expression, name] : terms) {
    if (expression->facts().zero)
      continue;
    const auto &form = detail::ExpressionAccess::impl(*expression).form;
    auto [negative, magnitude] = printed(form);
    std::string term;
    const auto stripped = negative ? SymEngine::neg(form) : form;
    if (SymEngine::eq(*stripped, *SymEngine::integer(1)))
      term = name;
    else if (SymEngine::is_a<SymEngine::Add>(*stripped))
      term = "(" + magnitude + ")*" + name;
    else
      term = magnitude + "*" + name;
    if (out.empty())
      out = (negative ? "-" : "") + term;
    else
      out += (negative ? " - " : " + ") + term;
  }
  if (out.empty())
    out = "0";
  out += " = " + detail::form_text(detail::ExpressionAccess::impl(right).form);
  // Powers as ^, as they are usually typed (SymEngine prints **).
  for (auto at = out.find("**"); at != std::string::npos;
       at = out.find("**", at + 1))
    out.replace(at, 2, "^");
  return out;
}

} // namespace

Equation parse_equation(std::string_view text) {
  Parser parser(text, Mode::equation,
                "Cannot read the equation " + quoted(text) + ": ");
  const auto [left, right] = parser.line();
  Evaluator evaluator(parser);
  const auto form = evaluator.difference(left, right);
  if (!has_u(form))
    parser.fail("it has no term with u");
  const auto at = [&](Term term) {
    return evaluator.coefficient(form, {term, Normal::one});
  };
  Equation equation{.a_xx = at(Term::u_xx),
                    .a_xy = at(Term::u_xy),
                    .a_yy = at(Term::u_yy),
                    .b_x = at(Term::u_x),
                    .b_y = at(Term::u_y),
                    .c = at(Term::u),
                    .m1 = at(Term::u_t),
                    .m2 = at(Term::u_tt),
                    .f = Expression()};
  equation.f = evaluator.coefficient(
      {{{Term::none, Normal::one}, SymEngine::neg(constant_part(form))}},
      {Term::none, Normal::one});
  return equation;
}

BoundaryCondition parse_condition(std::string_view text) {
  Parser parser(text, Mode::condition,
                "Cannot read the boundary condition " + quoted(text) + ": ");
  const auto [left, right] = parser.line();
  Evaluator evaluator(parser);
  const auto form = evaluator.difference(left, right);
  if (!has_u(form))
    parser.fail("it has no term with u");
  for (const auto &[key, coefficient] : form) {
    const auto [term, normal] = key;
    if (term == Term::u_xx || term == Term::u_xy || term == Term::u_yy ||
        term == Term::u_tt)
      parser.fail("it has a second derivative of u; a boundary condition has "
                  "only u, its first derivatives in space and u_t");
    if (normal != Normal::one && term != Term::u_x && term != Term::u_y)
      parser.fail("it uses the normal or tangent outside a derivative of u; "
                  "they may appear only as n·∇u, tau·∇u, u_n or u_tau");
  }
  const auto at = [&](Term term, Normal normal) {
    const auto it = form.find({term, normal});
    return it == form.end() ? RCP<const Basic>(SymEngine::integer(0))
                            : it->second;
  };
  // (beta_x + beta_n n_x - beta_tau n_y) u_x
  //   + (beta_y + beta_n n_y + beta_tau n_x) u_y.
  const auto normal = at(Term::u_x, Normal::n_x);
  const auto tangential = at(Term::u_y, Normal::n_x);
  const auto same = [](const RCP<const Basic> &a, const RCP<const Basic> &b) {
    const auto difference = SymEngine::sub(a, b);
    return zero_number(*difference) ||
           zero_number(*SymEngine::expand(difference));
  };
  if (!same(normal, at(Term::u_y, Normal::n_y)) ||
      !same(SymEngine::neg(tangential), at(Term::u_x, Normal::n_y)))
    parser.fail("it uses the normal or tangent in a derivative that is not "
                "beta_n u_n + beta_tau u_tau + beta_x u_x + beta_y u_y "
                "(for example n_x u_x alone, or (A ∇u)·n with a matrix "
                "that is not a multiple of the identity)");
  const auto coefficient = [&](const RCP<const Basic> &value) {
    return evaluator.coefficient({{{Term::none, Normal::one}, value}},
                                 {Term::none, Normal::one});
  };
  BoundaryCondition condition{
      .alpha = coefficient(at(Term::u, Normal::one)),
      .beta_n = coefficient(normal),
      .beta_tau = coefficient(tangential),
      .beta_x = coefficient(at(Term::u_x, Normal::one)),
      .beta_y = coefficient(at(Term::u_y, Normal::one)),
      .gamma = coefficient(at(Term::u_t, Normal::one)),
      .g = coefficient(SymEngine::neg(constant_part(form))),
      .priority = 0};
  return condition;
}

std::string equation_text(const Equation &equation) {
  const std::vector<std::pair<const Expression *, const char *>> terms = {
      {&equation.a_xx, "u_xx"}, {&equation.a_xy, "u_xy"},
      {&equation.a_yy, "u_yy"}, {&equation.b_x, "u_x"},
      {&equation.b_y, "u_y"},   {&equation.c, "u"},
      {&equation.m1, "u_t"},    {&equation.m2, "u_tt"}};
  check_names(terms, equation.f, Mode::equation, "equation");
  return line(terms, equation.f);
}

std::string condition_text(const BoundaryCondition &condition) {
  const std::vector<std::pair<const Expression *, const char *>> terms = {
      {&condition.alpha, "u"},        {&condition.beta_n, "u_n"},
      {&condition.beta_tau, "u_tau"}, {&condition.beta_x, "u_x"},
      {&condition.beta_y, "u_y"},     {&condition.gamma, "u_t"}};
  check_names(terms, condition.g, Mode::condition, "boundary condition");
  return line(terms, condition.g);
}

} // namespace pde2d::fem
