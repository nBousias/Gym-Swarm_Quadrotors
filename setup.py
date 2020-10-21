from distutils.core import setup


setup(
    name='swarm_quadrotors',
    packages=['flightsim'],
    version='0.1',
    install_requires=['gym','cvxopt','matplotlib','numpy','scipy','timeout_decorator'])