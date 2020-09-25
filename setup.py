from distutils.core import setup

setup(name='robosim',
      version='0.1.0',
      author='Mateus Machado, Felipe Martins',
      author_email='mgm4, fbm2 (@cin.ufpe.br)',
      description='RoboSim simulator for VSS and SSL',
      license='MIT',
      keywords=('Robocup '
                'VSS '
                'SSL '
                ),
      requires=['numpy', 'gym'],
      packages=['robosim']
      )
