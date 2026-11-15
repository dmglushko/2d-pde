// The server's JSON API (docs/app.md, "The API").

export class ApiError extends Error {
  constructor(error) {
    super(error?.message ?? "the request failed");
    this.stage = error?.stage ?? "request";
    this.kind = error?.kind ?? "internal";
  }
}

async function request(path, body) {
  let response;
  try {
    response = await fetch(path, body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  } catch {
    throw new ApiError({ stage: "request", kind: "internal", message: "The app server does not answer. Is pde2d_app still running?" });
  }
  const data = await response.json().catch(() => null);
  if (!response.ok) throw new ApiError(data?.error ?? { message: `${response.status} ${response.statusText}` });
  return data;
}

export const api = {
  catalog: () => request("/api/catalog"),
  examples: () => request("/api/examples"),
  example: (name) => request(`/api/examples/${encodeURIComponent(name)}`),
  geometry: (caseData) => request("/api/geometry", caseData),
  mesh: (caseData) => request("/api/mesh", caseData),
  problem: (caseData) => request("/api/problem", caseData),
  assemble: (caseData) => request("/api/assemble", caseData),
  field: (caseData, field) => request("/api/field", { case: caseData, field }),
  solve: (caseData) => request("/api/solve", caseData),
};

// Asks the server for an export of the case and saves it as a file.
export async function downloadExport(kind, caseData) {
  let response;
  try {
    response = await fetch(`/api/export/${kind}`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(caseData) });
  } catch {
    throw new ApiError({ stage: "export", message: "The app server does not answer." });
  }
  if (!response.ok) {
    const data = await response.json().catch(() => null);
    throw new ApiError(data?.error ?? { stage: "export", message: response.statusText });
  }
  saveBlob(await response.blob(), kind);
}

export function saveBlob(blob, name) {
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
