import sys

try:
    from skbuild import setup
except ImportError:
    print('Please update pip, you need pip 10 or greater,\n'
          ' or you need to install the PEP 518 requirements in pyproject.toml yourself', file=sys.stderr)
    raise

setup(
    name="rc-robosim",
    version="0.0.1",
    description="a minimal example package (with pybind11)",
    author='Felipe Martins',
    packages=['robosim'],
    package_dir={'': 'src'},
    url="https://github.com/robocin/rSim",
    author_email="fbm2@cin.ufpe.br"
)
