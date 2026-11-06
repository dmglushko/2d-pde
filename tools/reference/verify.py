"""Independent P1 forms; no C++ matrices are used as the reference operator.

https://scikit-fem.readthedocs.io/en/latest/
https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
"""
import json
import sys
import subprocess
from pathlib import Path
import numpy as np
import scipy
import skfem
import matplotlib
from scipy.sparse.linalg import spsolve, splu
from scipy.integrate import solve_ivp
from skfem import MeshTri, Basis, FacetBasis, ElementTriP1, BilinearForm, LinearForm, asm

exporter, directory = sys.argv[1:]
subprocess.run([exporter, directory], check=True)
data = json.loads((Path(directory) / 'reference.json').read_text())
if data['openmp']:
    for workers in (2, 4):
        child = str(Path(directory) / str(workers))
        subprocess.run([exporter, child, str(workers)], check=True)
        assert json.loads((Path(child) / 'reference.json').read_text()) == data, workers
points = np.array(data['nodes']).T
mesh = MeshTri(points, np.array(data['triangles']).T, sort_t=False)
basis = Basis(mesh, ElementTriP1(), intorder=8)
# Explicit coordinate mapping: C++ node -> scikit-fem DOF, checked bijective.
mapping = np.array([np.flatnonzero(np.all(np.isclose(basis.doflocs, p[:, None], atol=1e-14, rtol=0), axis=0)).item() for p in points.T])
assert len(set(mapping)) == points.shape[1]
assert np.array_equal(mesh.t, np.array(data['triangles']).T)

def facets(side):
    axis, value = {'top': (1, 1), 'bottom': (1, 0)}[side]
    return mesh.facets_satisfying(lambda x: np.isclose(x[axis], value))

@BilinearForm
def stationary(u, v, w):
    x, y = w.x
    # PDE is NONdivergence form: -D:Hess(u)+b.grad(u)+u.
    # Weak convection is b+div(D)=(1.3,.8), not b alone.
    return ((2+x)*u.grad[0]*v.grad[0]+.2*(u.grad[0]*v.grad[1]+u.grad[1]*v.grad[0])
            +(3+y)*u.grad[1]*v.grad[1]+(1.3*u.grad[0]+.8*u.grad[1])*v+u*v)
@BilinearForm
def weighted_mass(u, v, w): return (1+w.x[0])*u*v
@BilinearForm
def mass(u, v, w): return u*v
@BilinearForm
def laplace(u, v, w): return u.grad[0]*v.grad[0]+u.grad[1]*v.grad[1]
@LinearForm
def load(v, w): return (1+w.x[0]+2*w.x[1])*v
@LinearForm
def top_load(v, w): return (2+w.x[0])*v
@LinearForm
def bottom_load(v, w): return (.5+w.x[0])*v
@LinearForm
def base_load(v, w): return (1+w.x[0]+w.x[1])*v

top = FacetBasis(mesh, ElementTriP1(), facets=facets('top'), intorder=8)
bottom = FacetBasis(mesh, ElementTriP1(), facets=facets('bottom'), intorder=8)
K = asm(stationary, basis)+asm(mass, top)
C = asm(weighted_mass, basis)+.4*asm(mass, top)
F = asm(load, basis)+asm(top_load, top)+asm(bottom_load, bottom)
metrics = {'execution_workers_checked': [0, 2, 4] if data['openmp'] else [0]}
for key, reference in [('K', K), ('C', C)]:
    error = np.max(np.abs(reference.toarray()[mapping][:, mapping]-data[key]))
    metrics[key+'_max_error'] = float(error)
    assert error < 2e-12, (key, error)
metrics['F_max_error'] = float(np.max(np.abs(F[mapping]-data['F'])))
assert metrics['F_max_error'] < 2e-12
D = np.flatnonzero(np.isclose(basis.doflocs[0], 0)|np.isclose(basis.doflocs[0], 1))
I = np.setdiff1d(np.arange(basis.N), D)
u = np.zeros(basis.N); u[D] = 1+basis.doflocs[1, D]
u[I] = spsolve(K[I][:, I], F[I]-K[I][:, D]@u[D])
metrics['stationary_max_error'] = float(np.max(np.abs(u[mapping]-data['stationary'])))
assert metrics['stationary_max_error'] < 2e-11

boundary = FacetBasis(mesh, ElementTriP1(), intorder=8)
M = asm(mass, basis); B = asm(mass, boundary); K = asm(laplace, basis)+B
f = asm(base_load, basis); g = asm(base_load, boundary)
u0 = 1+basis.doflocs.sum(axis=0)
for method in range(3):
    wave = method == 2
    C = (.2 if wave else 1)*M+.4*B
    factor = splu((M if wave else C).tocsc())
    def rhs(t, z):
        force = np.sin(t)*f+(1+t)*g
        if wave:
            u, v = z[:basis.N], z[basis.N:]
            return np.r_[v, factor.solve(force-K@u-C@v)]
        return factor.solve(force-K@z)
    initial = np.r_[u0, -u0] if wave else u0
    # Two unrelated adaptive algorithms agree well below tested discretization errors.
    solutions = [solve_ivp(rhs, (0, .2), initial, method=m, rtol=2e-12, atol=2e-13) for m in ('DOP853', 'Radau')]
    assert all(s.success for s in solutions)
    oracle_gap = np.max(np.abs(solutions[0].y[:, -1]-solutions[1].y[:, -1]))
    assert oracle_gap < 2e-10
    ref = solutions[0].y[:basis.N, -1]
    errors = [float(np.max(np.abs(np.array(r['u'])-ref[mapping]))) for r in data['runs'] if r['method'] == method]
    assert errors[-1] < (.02 if method == 0 else .002), errors
    assert all(a/b > (1.7 if method == 0 else 3.4) for a, b in zip(errors, errors[1:])), errors
    metrics[str(method)] = {'dt': [.04, .02, .01], 'max_errors': errors, 'oracle_gap': float(oracle_gap), 'operation_counts': [{k: r[k] for k in ('assembly_calls', 'linear_solves', 'solver_reuses')} for r in data['runs'] if r['method'] == method]}
metrics['versions'] = {'python': sys.version, 'numpy': np.__version__, 'scipy': scipy.__version__, 'scikit-fem': skfem.__version__, 'matplotlib': matplotlib.__version__}
(Path(directory) / 'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
print(json.dumps(metrics, indent=2))
