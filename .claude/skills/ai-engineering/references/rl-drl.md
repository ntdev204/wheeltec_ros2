# Reinforcement Learning & Deep RL - Details

## Classical RL Algorithms

### Q-Learning (Tabular)

```python
Q(s,a) ← Q(s,a) + α[r + γ max Q(s',a') - Q(s,a)]
```

- **Type**: Off-policy, value-based
- **Environment**: Discrete state and action spaces
- **Convergence**: Guaranteed with proper conditions
- **Use**: Small state spaces (e.g., grid world)

### SARSA (State-Action-Reward-State-Action)

```python
Q(s,a) ← Q(s,a) + α[r + γQ(s',a') - Q(s,a)]
```

- **Type**: On-policy
- **Difference from Q-learning**: Uses actual next action, not max
- **Use**: When on-policy behavior matters

### Policy Gradient

```python
∇θ J(θ) = E[∇θ log πθ(a|s) × reward]
```

- **Type**: Direct policy optimization
- **Advantage**: Handles continuous actions
- **Disadvantage**: High variance

## Deep Reinforcement Learning

### DQN (Deep Q-Network)

**Innovations:**

1. **Experience Replay**: Break correlation in sequences
2. **Target Network**: Stabilize learning

```python
Loss = (r + γ max Q_target(s',a') - Q(s,a))²
```

**Variants:**

- **Double DQN**: Reduce overestimation
- **Dueling DQN**: Separate value and advantage
- **Rainbow DQN**: Combines multiple improvements

**Use Cases:**

- Atari games
- Discrete action spaces
- Sample efficiency important

### PPO (Proximal Policy Optimization)

**Key Idea**: Trust region optimization

```python
L^CLIP(θ) = E[min(r_t(θ)A_t, clip(r_t(θ), 1-ε, 1+ε  )A_t)]
where r_t(θ) = πθ(a|s) / πθ_old(a|s)
```

**Strengths:**

- Stable training
- Good sample efficiency
- Easy to implement

**Use Cases:**

- Continuous control
- Robotics
- General-purpose RL

### A3C / A2C (Advantage Actor-Critic)

**A3C**: Asynchronous Advantage Actor-Critic

- **Parallelization**: Multiple workers
- **No replay buffer**: On-policy

**A2C**: Advantage Actor-Critic (synchronous)

- **Simpler**: Synchronous updates
- **More stable**: than A3C

**Use**: Multi-threaded environments

### SAC (Soft Actor-Critic)

**Key Features:**

- **Maximum entropy**: Encourage exploration
- **Off-policy**: Sample efficient
- **Continuous actions**: Stochastic policy

```python
J(π) = E[r(s,a) + α H(π(·|s))]
where H = entropy
```

**Use Cases:**

- Continuous control
- Robotics
- When sample efficiency critical

### TD3 (Twin Delayed DDPG)

**Improvements over DDPG:**

1. **Twin Q-networks**: Reduce overestimation
2. **Delayed policy updates**: Stabilize training
3. **Target policy smoothing**: Reduce variance

**Use**: Continuous control, alternative to SAC

## Environment Design

### Markov Decision Process (MDP)

- **State space** (S)
- **Action space** (A)
- **Transition dynamics** P(s'|s,a)
- **Reward function** R(s,a,s')
- **Discount factor** γ

### Reward Shaping

**Principles:**

- Sparse rewards → Reward shaping
- Avoid reward hacking
- Balance exploration vs exploitation

**Techniques:**

- Intrinsic motivation (curiosity)
- Curriculum learning
- Hindsight Experience Replay (HER)

## Multi-Agent RL

### Approaches

- **Independent learners**: Each agent learns separately
- **Centralized training, decentralized execution**: Share info during training
- **Communication**: Agents exchange messages

### Algorithms

- **MADDPG**: Multi-agent DDPG
- **QMIX**: Value decomposition
- **CommNet**: Communication networks

## Exploration Strategies

| Strategy                 | Description                      | Use Case          |
| ------------------------ | -------------------------------- | ----------------- |
| **ε-greedy**             | Random action with probability ε | Simple, DQN       |
| **Boltzmann**            | Softmax over Q-values            | Temperature-based |
| **UCB**                  | Upper confidence bound           | Bandit problems   |
| **Intrinsic Motivation** | Curiosity-driven                 | Sparse rewards    |
| **Noisy Networks**       | Parametric noise in weights      | DQN variants      |

## Practical Implementation

### Training Loop (PyTorch + Gymnasium)

```python
import gymnasium as gym
env = gym.make('CartPole-v1')

for episode in range(num_episodes):
    state, info = env.reset()
    done = False

    while not done:
        action = agent.select_action(state)
        next_state, reward, terminated, truncated, info = env.step(action)
        agent.store_transition(state, action, reward, next_state, done)
        agent.train()
        state = next_state
        done = terminated or truncated
```

### Hyperparameters

**Common ranges:**

- Learning rate: 1e-4 to 1e-3
- Discount factor (γ): 0.95 to 0.99
- Batch size: 32 to 256
- Replay buffer: 10k to 1M transitions

## Debugging RL

### Common Issues

- **Not learning**: Check reward scale, learning rate
- **Unstable**: Reduce learning rate, use target networks
- **Slow convergence**: Increase network capacity, tune hyperparameters
- **Overfitting**: Add noise, increase env diversity

### Logging

- Episode reward (mean, std, max, min)
- Q-value estimates
- Loss curves
- Exploration rate

## RL Frameworks

| Framework             | Language | Features            |
| --------------------- | -------- | ------------------- |
| **Stable Baselines3** | Python   | High-level, PyTorch |
| **RLlib**             | Python   | Scalable, Ray       |
| **TF-Agents**         | Python   | TensorFlow          |
| **Acme**              | Python   | DeepMind, modular   |
| **OpenAI Gym**        | Python   | Environments        |
| **Gymnasium**         | Python   | Maintained Gym fork |

## Benchmarks

### Atari

- 57 games from Atari 2600
- Pixel observations
- Standard benchmark for DQN

### MuJoCo

- Physics simulation
- Continuous control
- Benchmark for PPO, SAC, TD3

### DM Control Suite

- DeepMind control tasks
- Physics-based
- Alternative to MuJoCo
