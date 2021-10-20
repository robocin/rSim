import sys

try:
    from skbuild import setup
except ImportError:
    print('Please update pip, you need pip 10 or greater,\n'
          ' or you need to install the PEP 518 requirements in pyproject.toml yourself', file=sys.stderr)
    raise

setup(
    name="rc-robosim",
    url="https://github.com/robocin/rSim",
    version="0.0.8a0",
    description="SSL and VSS robot soccer simulator",
    author='Felipe Martins',
    author_email="fbm2@cin.ufpe.br",
    packages=['robosim'],
    package_dir={'': 'src'},
    cmake_install_dir='src/robosim'
)
