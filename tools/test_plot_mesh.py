"""Topology prefix validation and fresh C++ export rendering, isolated in CTest."""
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch
import plot_mesh

prefix = 'PDE2D_MESH {}\nORDER 1\nNODES 3\n0 0\n1 0\n0 1\nELEMENTS 1\n3 0 1 2\nBOUNDARY_EDGES 3\n{}'
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)/'fixture.pde2d'
    for version in range(1, 5):
        edges = '0 1\n1 2\n2 0\n' if version == 1 else '0 1 inlet\n1 2 -\n2 0 wall\n'
        valid = prefix.format(version, edges)
        path.write_text(valid + ('ignored metadata, intentionally unvalidated' if version >= 3 else ''))
        assert len(plot_mesh.read_mesh(path)[0]) == 3
        for bad in (valid[:30], valid.replace('0 0\n', 'nan 0\n', 1), valid.replace('0 0\n', 'inf 0\n', 1),
                    valid.replace('3 0 1 2', '3 0 1 9'), valid.replace('3 0 1 2', '3 -1 1 2'),
                    valid.replace('NODES 3', 'NODES -3'), valid.replace('3 0 1 2', '2 0 1'),
                    valid.replace('ORDER 1', 'ORDER 2'), valid.replace('PDE2D_MESH '+str(version), 'PDE2D_MESH 99'),
                    valid.rsplit('\n', 2)[0]):
            path.write_text(bad)
            try:
                plot_mesh.read_mesh(path)
            except ValueError:
                pass
            else:
                raise AssertionError('Accepted malformed topology: '+bad)
    fresh = sorted(Path(sys.argv[1]).glob('*.pde2d'))
    assert len(fresh) == 19
    assert all(p.read_text().startswith('PDE2D_MESH 4') for p in fresh)
    calls = []
    original = plot_mesh.read_mesh
    def read(path):
        calls.append(path)
        return original(path)
    output = Path.cwd()/'png'
    with patch.object(plot_mesh, 'read_mesh', read), patch.object(sys, 'argv', ['plot_mesh', *map(str, fresh), '--output-dir', str(output)]):
        plot_mesh.main()
    assert calls == fresh, calls
    assert len(list(output.glob('*.png'))) == 20
    assert all(p.stat().st_size > 1000 for p in output.glob('*.png'))
    print('v1-v4, malformed topology, one parse per file, 19 fresh PNGs + gallery passed')
