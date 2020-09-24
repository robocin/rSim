from distutils.core import setup

setup(name = 'robosim',
      version='0.1.0',
      author='Mateus Goncalves, ',
      author_email='mgm4@cin.ufpe.br',
      description='RoboSim simulator for VSS and SSL',
      license='MIT',
      keywords=('Robocup '
                'VSS '
              ),
      requires=['numpy'],
      packages=['robosim']
)