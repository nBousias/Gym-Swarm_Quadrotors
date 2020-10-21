from gym.envs.registration import register

register(
    id='swarm_quadrotors-v0.1',
    entry_point='swarm_quadrotors.envs:Swarm',
)

