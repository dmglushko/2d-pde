#pragma once

#include "curve_length.hpp"
#include <compare>
#include <cstdint>
#include <limits>
#include <map>
#include <sstream>
#include <mutex>
#include <unordered_map>

namespace pde2d::discretization {

// Construction input: consecutive boundary curves with one tag.
struct CurveRegion {
  std::string name;
  std::vector<CurveSegment> segments;
};

struct CurveId {
  std::uint64_t value = 0;
  auto operator<=>(const CurveId &) const = default;
};
struct BoundaryCurve {
  CurveId id;
  CurveSegment segment;
  std::string region;
};
struct RegionInfo {
  std::string name;
  std::vector<CurveId> curves; // Boundary order, starting at the stored seam.
  std::vector<std::vector<CurveId>>
      pieces; // Each piece follows curve direction.
  bool connected() const { return pieces.size() == 1; }
};

// One ordered closed boundary; named membership is independent of connectivity.
// Concurrent const queries and copying a read-only source are supported,
// including simultaneous cache misses. Each copy owns its cache. Mutation
// (including move from an object) requires external synchronization. Any edit
// may invalidate references returned by curves()/curve(); cache locking does
// not protect them.
class CurveGeometry : public Geometry2D {
public:
  CurveGeometry() = default;
  explicit CurveGeometry(std::vector<CurveRegion> regions) {
    setRegions(std::move(regions));
  }

  const std::vector<BoundaryCurve> &curves() const { return curves_; }
  const BoundaryCurve &curve(CurveId id) const {
    auto it = ids_.find(id.value);
    if (it != ids_.end())
      return curves_[it->second];
    throw std::out_of_range("Unknown or retired curve ID " +
                            std::to_string(id.value));
  }
  std::vector<RegionInfo> regionGroups() const { return groups_; }
  RegionInfo region(const std::string &name) const { return regionInfo(name); }
  CurveId curveIdAt(const std::string &name, std::size_t index) const {
    const auto &ids = regionInfo(name).curves;
    if (index >= ids.size())
      throw std::out_of_range("Invalid curve index in region " + name);
    return ids[index];
  }
  CurveId curveIdAt(std::size_t regionIndex, std::size_t segmentIndex) const {
    const auto &runs = regionRuns();
    if (regionIndex >= runs.size() || segmentIndex >= runs[regionIndex].size())
      throw std::out_of_range("Invalid region or segment index");
    return runs[regionIndex][segmentIndex];
  }
  double length(CurveId id) const {
    const auto &c = curve(id);
    std::lock_guard lock(lengths_.mutex);
    auto it = lengths_.find(id);
    if (it == lengths_.end())
      it = lengths_.emplace(id, curveLength(c.segment)).first;
    return it->second;
  }
  double regionLength(const std::string &name) const {
    double total = 0;
    for (auto id : regionInfo(name).curves)
      total += length(id);
    if (!std::isfinite(total))
      throw std::invalid_argument(
          "Region length exceeds supported numeric scale");
    return total;
  }

  // Explicit replacement resets edits and assigns fresh IDs. Shape setters
  // instead use rebuildRegions(), which preserves source identities/history.
  void setRegions(std::vector<CurveRegion> regions) {
    CurveGeometry next = *this;
    next.curves_ = next.makeCurves(regions);
    next.base_ = next.curves_;
    next.edits_.clear();
    next.lengths_.clear();
    commit(std::move(next));
  }
  void renameRegion(const std::string &source, const std::string &target) {
    (void)region(source);
    validateName(target);
    if (source == target)
      return;
    CurveGeometry next = *this;
    RelabelEdit edit{{}, target, source};
    next.apply(edit);
    next.edits_.push_back(std::move(edit));
    commit(std::move(next));
  }
  void mergeRegionInto(const std::string &source, const std::string &target) {
    (void)region(target);
    renameRegion(source, target);
  }
  void moveCurve(CurveId id, const std::string &target) {
    (void)curve(id);
    relabel({id}, target);
  }

  std::vector<CurveId> splitCurve(CurveId id, double fraction,
                                  const std::string &first,
                                  const std::string &second) {
    return splitCurve(id, std::vector<double>{fraction},
                      std::vector<std::string>{first, second});
  }
  std::vector<CurveId> splitCurve(CurveId id,
                                  const std::vector<double> &fractions,
                                  const std::vector<std::string> &names) {
    (void)curve(id);
    return split({id}, fractions, names);
  }
  std::vector<CurveId> splitRegion(const std::string &name, double fraction,
                                   const std::string &first,
                                   const std::string &second) {
    return splitRegion(name, std::vector<double>{fraction},
                       std::vector<std::string>{first, second});
  }
  std::vector<CurveId> splitRegion(const std::string &name,
                                   const std::vector<double> &fractions,
                                   const std::vector<std::string> &names) {
    auto r = region(name);
    if (!r.connected()) {
      std::ostringstream message;
      message << "Region '" << name
              << "' is disconnected; split a curve by ID. Available IDs:";
      for (auto id : r.curves)
        message << ' ' << id.value;
      throw std::invalid_argument(message.str());
    }
    return split(r.pieces.front(), fractions, names, name);
  }

  // Positional API: regions are consecutive tagged runs; t is a segment's
  // mathematical parameter, NOT a length fraction. Names cover the whole run.
  void splitRegion(std::size_t regionIndex, std::size_t segmentIndex, double t,
                   const std::string &first, const std::string &second) {
    const auto &runs = regionRuns();
    if (regionIndex >= runs.size() || segmentIndex >= runs[regionIndex].size())
      throw std::out_of_range("Invalid region or segment index");
    validateFractions({t}, {first, second});
    CurveGeometry next = *this;
    SplitEdit edit{runs[regionIndex], {t}, {first, second}, {}, true,
                   segmentIndex,      {}};
    next.apply(edit);
    next.edits_.push_back(std::move(edit));
    commit(std::move(next));
  }
  void mergeRegions(std::size_t first, std::size_t count,
                    const std::string &name) {
    const auto &runs = regionRuns();
    if (count < 2 || first >= runs.size() || count > runs.size() - first)
      throw std::out_of_range("Invalid region range");
    std::vector<CurveId> ids;
    for (std::size_t i = first; i < first + count; ++i)
      ids.insert(ids.end(), runs[i].begin(), runs[i].end());
    relabel(ids, name);
  }

protected:
  // Built-in shape definitions provide the same source curves in the same
  // semantic order when dimensions change. Topology changes with edits fail.
  void rebuildRegions(const std::vector<CurveRegion> &regions) {
    CurveGeometry next = *this;
    auto rebuilt = next.makeCurves(regions);
    bool same = rebuilt.size() == base_.size();
    if (same)
      for (std::size_t i = 0; i < rebuilt.size(); ++i)
        if (rebuilt[i].region != base_[i].region ||
            rebuilt[i].segment.index() != base_[i].segment.index()) {
          same = false;
          break;
        }
    if (!same && !edits_.empty())
      throw std::invalid_argument(
          "Shape topology changed; cannot preserve region edits");
    if (same)
      for (std::size_t i = 0; i < rebuilt.size(); ++i)
        rebuilt[i].id = base_[i].id;
    next.base_ = rebuilt;
    next.curves_ = std::move(rebuilt);
    next.lengths_.clear();
    next.reindex();
    // Replaying mutates the generated-ID maps only on this temporary copy.
    try {
      for (auto &edit : next.edits_)
        std::visit([&](auto &e) { next.apply(e); }, edit);
    } catch (const std::out_of_range &e) {
      throw std::invalid_argument(
          std::string("Resize cannot preserve an edited curve: ") + e.what());
    }
    commit(std::move(next));
  }

private:
  struct RelabelEdit {
    std::vector<CurveId> source;
    std::string target;
    std::string sourceName;
  };
  struct SplitEdit {
    std::vector<CurveId> source;
    std::vector<double> fractions;
    std::vector<std::string> names;
    // A fragment is identified by its input curve and output interval.
    std::map<std::pair<CurveId, std::size_t>, CurveId> generated;
    bool parameter = false;
    std::size_t parameterSegment = 0;
    std::string sourceName;
  };
  using Edit = std::variant<RelabelEdit, SplitEdit>;
  std::vector<BoundaryCurve> curves_, base_;
  std::vector<Edit> edits_;
  // The mutex belongs to the cache value, never shared by geometry copies.
  struct LengthCache : std::map<CurveId, double> {
    mutable std::mutex mutex;
    LengthCache() = default;
    LengthCache(const LengthCache &other) : std::map<CurveId, double>() {
      std::lock_guard lock(other.mutex);
      std::map<CurveId, double>::operator=(other);
    }
    LengthCache &operator=(const LengthCache &other) {
      if (this != &other) {
        std::scoped_lock lock(mutex, other.mutex);
        std::map<CurveId, double>::operator=(other);
      }
      return *this;
    }
    LengthCache(LengthCache &&other) noexcept
        : std::map<CurveId, double>(std::move(other)) {}
    LengthCache &operator=(LengthCache &&other) noexcept {
      std::map<CurveId, double>::operator=(std::move(other));
      return *this;
    }
  };
  mutable LengthCache lengths_;
  std::unordered_map<std::uint64_t, std::size_t> ids_;
  std::unordered_map<std::string, std::size_t> regionIndex_;
  std::vector<RegionInfo> groups_;
  std::vector<std::vector<CurveId>> runs_;

  const RegionInfo &regionInfo(const std::string &name) const {
    auto it = regionIndex_.find(name);
    if (it == regionIndex_.end())
      throw std::out_of_range("Unknown region: " + name);
    return groups_[it->second];
  }
  void reindex() {
    ids_.clear();
    regionIndex_.clear();
    groups_.clear();
    runs_.clear();
    for (std::size_t i = 0; i < curves_.size(); ++i) {
      const auto &c = curves_[i];
      ids_.emplace(c.id.value, i);
      if (i == 0 || curves_[i - 1].region != c.region)
        runs_.emplace_back();
      runs_.back().push_back(c.id);
      auto [it, added] = regionIndex_.emplace(c.region, groups_.size());
      if (added)
        groups_.push_back({c.region, {}, {}});
      auto &r = groups_[it->second];
      r.curves.push_back(c.id);
      if (i == 0 || curves_[i - 1].region != c.region)
        r.pieces.emplace_back();
      r.pieces.back().push_back(c.id);
    }
    if (curves_.size() > 1 && curves_.front().region == curves_.back().region) {
      auto &pieces = groups_[regionIndex_.at(curves_.front().region)].pieces;
      if (pieces.size() > 1) {
        // Old traversal starts at the first transition, so the seam piece is
        // last.
        auto first = std::move(pieces.front());
        pieces.erase(pieces.begin());
        pieces.back().insert(pieces.back().end(), first.begin(), first.end());
      }
    }
  }
  std::uint64_t nextId_ = 1;

  void commit(CurveGeometry &&next) {
    next.reindex();
    curves_.swap(next.curves_);
    base_.swap(next.base_);
    edits_.swap(next.edits_);
    ids_.swap(next.ids_);
    regionIndex_.swap(next.regionIndex_);
    groups_.swap(next.groups_);
    runs_.swap(next.runs_);
    lengths_.swap(next.lengths_);
    std::swap(nextId_, next.nextId_);
  }
  CurveId allocateId() {
    if (nextId_ == std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("Curve ID space exhausted");
    return {nextId_++};
  }
  static void validateName(const std::string &name) {
    if (name.empty() || !boundary_detail::valid_tag(name))
      throw std::invalid_argument("Region names require ASCII letters, digits, "
                                  "_, . or -; '-' is reserved");
  }
  std::vector<BoundaryCurve>
  makeCurves(const std::vector<CurveRegion> &regions) {
    if (regions.empty())
      throw std::invalid_argument("Curve geometry requires regions");
    std::vector<BoundaryCurve> records;
    for (const auto &r : regions) {
      validateName(r.name);
      if (r.segments.empty())
        throw std::invalid_argument("Curve regions require segments");
      for (const auto &segment : r.segments) {
        curve_detail::validateCurve(segment);
        records.push_back({allocateId(), segment, r.name});
      }
    }
    return records;
  }
  const std::vector<std::vector<CurveId>> &regionRuns() const { return runs_; }
  static void validateFractions(const std::vector<double> &fractions,
                                const std::vector<std::string> &names) {
    if (fractions.empty() || names.size() != fractions.size() + 1)
      throw std::invalid_argument(
          "Splits require N fractions and N+1 region names");
    double previous = 0;
    for (double f : fractions) {
      if (!std::isfinite(f) || !(f > previous && f < 1))
        throw std::invalid_argument("Split fractions must be finite, strictly "
                                    "increasing and between 0 and 1");
      previous = f;
    }
    for (const auto &n : names)
      validateName(n);
  }
  void relabel(const std::vector<CurveId> &ids, const std::string &name) {
    validateName(name);
    std::vector<CurveId> changed;
    for (auto id : ids)
      if (curve(id).region != name)
        changed.push_back(id);
    if (changed.empty())
      return;
    CurveGeometry next = *this;
    RelabelEdit edit{std::move(changed), name, {}};
    next.apply(edit);
    next.edits_.push_back(std::move(edit));
    commit(std::move(next));
  }
  void apply(const RelabelEdit &edit) {
    auto source =
        edit.sourceName.empty() ? edit.source : region(edit.sourceName).curves;
    // Validate all targets before modifying any of them.
    for (auto id : source)
      (void)curve(id);
    for (auto id : source)
      curves_[ids_.at(id.value)].region = edit.target;
    reindex();
  }
  std::vector<CurveId> split(const std::vector<CurveId> &source,
                             const std::vector<double> &fractions,
                             const std::vector<std::string> &names,
                             const std::string &sourceName = {}) {
    validateFractions(fractions, names);
    CurveGeometry next = *this;
    SplitEdit edit{source, fractions, names, {}, false, 0, sourceName};
    auto ids = next.apply(edit);
    next.edits_.push_back(std::move(edit));
    commit(std::move(next));
    return ids;
  }
  std::vector<CurveId> apply(SplitEdit &edit) {
    if (!edit.sourceName.empty()) {
      auto r = region(edit.sourceName);
      if (!r.connected())
        throw std::invalid_argument(
            "Cannot replay split of disconnected region " + edit.sourceName);
      edit.source = r.pieces.front();
    }
    std::vector<double> lengths;
    double total = 0;
    for (auto id : edit.source) {
      double l = length(id);
      if (!(l > 0))
        throw std::invalid_argument(
            "Cannot split a selection containing a zero-length curve");
      lengths.push_back(l);
      total += l;
    }
    if (!(total > 0) || !std::isfinite(total))
      throw std::invalid_argument("Invalid total region length");
    auto fractions = edit.fractions;
    if (edit.parameter) {
      double before = 0;
      for (std::size_t i = 0; i < edit.parameterSegment; ++i)
        before += lengths[i];
      fractions[0] =
          (before +
           curveLength(curve(edit.source[edit.parameterSegment]).segment,
                       fractions[0])) /
          total;
    }
    std::map<CurveId, std::vector<BoundaryCurve>> replacements;
    std::map<CurveId, double> measured;
    std::vector<CurveId> result;
    std::vector<bool> represented(edit.names.size(), false);
    double accumulated = 0;
    for (std::size_t i = 0; i < edit.source.size(); ++i) {
      const auto &original = curve(edit.source[i]);
      double lo = accumulated / total;
      accumulated += lengths[i];
      double hi = (i + 1 == edit.source.size()) ? 1 : accumulated / total;
      if (!(hi > lo))
        throw std::invalid_argument(
            "Selected curve lengths exceed supported relative numeric scale");
      // Snap only at existing junctions, relative to the selected length.
      // Never silently merge two user-specified cuts.
      constexpr double junctionTolerance = 2e-11;
      std::vector<std::pair<double, double>>
          cuts; // global fraction, local parameter
      for (double f : fractions)
        if (f > lo && f < hi) {
          if ((i > 0 && f - lo <= junctionTolerance) ||
              (i + 1 < edit.source.size() && hi - f <= junctionTolerance))
            continue;
          double t = edit.parameter && i == edit.parameterSegment
                         ? edit.fractions[0]
                         : curveParameterAtLengthFraction(original.segment,
                                                          (f - lo) / (hi - lo));
          cuts.push_back({f, t});
        }
      std::vector<BoundaryCurve> pieces;
      CurveSegment remaining = original.segment;
      double previousT = 0, previousF = lo;
      for (std::size_t k = 0; k <= cuts.size(); ++k) {
        double endF = k < cuts.size() ? cuts[k].first : hi;
        CurveSegment piece = remaining;
        if (k < cuts.size()) {
          double t = (cuts[k].second - previousT) / (1 - previousT);
          auto halves = discretization::splitCurve(remaining, t);
          piece = std::move(halves.first);
          remaining = std::move(halves.second);
          previousT = cuts[k].second;
        }
        double middle = previousF + (endF - previousF) / 2;
        std::size_t part =
            std::upper_bound(fractions.begin(), fractions.end(), middle) -
            fractions.begin();
        represented[part] = true;
        CurveId id = original.id;
        // Once this edit has subdivided an input curve, never expose its
        // retired input ID again if a resize moves the cut onto a neighbor.
        // The surviving input/interval fragment keeps its own identity.
        const auto firstGenerated =
            edit.generated.lower_bound({original.id, 0});
        bool wasSplit = firstGenerated != edit.generated.end() &&
                        firstGenerated->first.first == original.id;
        if (!cuts.empty() || wasSplit) {
          auto key = std::make_pair(original.id, part);
          auto it = edit.generated.find(key);
          if (it == edit.generated.end())
            it = edit.generated.emplace(key, allocateId()).first;
          id = it->second;
        }
        double pieceLength = cuts.empty() ? lengths[i] : curveLength(piece);
        if (!(pieceLength > 0))
          throw std::invalid_argument(
              "Split portion is below supported geometric precision");
        measured[id] = pieceLength;
        pieces.push_back({id, std::move(piece), edit.names[part]});
        result.push_back(id);
        previousF = endF;
      }
      replacements.emplace(original.id, std::move(pieces));
    }
    if (std::find(represented.begin(), represented.end(), false) !=
        represented.end())
      throw std::invalid_argument("Split fractions are too close to a junction "
                                  "to resolve distinct portions");
    std::vector<BoundaryCurve> next;
    for (const auto &c : curves_) {
      auto it = replacements.find(c.id);
      if (it == replacements.end())
        next.push_back(c);
      else
        next.insert(next.end(), it->second.begin(), it->second.end());
    }
    curves_ = std::move(next);
    for (auto id : edit.source)
      lengths_.erase(id);
    lengths_.insert(measured.begin(), measured.end());
    reindex();
    return result;
  }
};
} // namespace pde2d::discretization
