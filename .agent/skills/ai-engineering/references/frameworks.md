# AI Frameworks - Deep Dive

## PyTorch

### Strengths

- Dynamic computational graphs (define-by-run)
- Pythonic and intuitive API
- Strong research community
- Excellent debugging (standard Python debugger works)
- TorchScript for production deployment
- Growing ecosystem (torchvision, torchaudio, etc.)

### When to Use

- Research and prototyping
- Custom architecture development
- When you need flexibility and debugging
- NLP tasks (transformers ecosystem)
- Academic research

### Production Deployment

- **TorchServe**: Official serving framework
- **TorchScript**: Optimize and serialize models
- **ONNX Export**: Cross-platform deployment

## TensorFlow

### Strengths

- Mature production ecosystem
- TensorFlow Serving for deployment
- TensorFlow Lite for mobile/edge
- TensorFlow.js for browser
- Strong distributed training support
- TensorBoard for visualization

### When to Use

- Production deployment at scale
- Mobile and edge deployment
- When you need proven, enterprise-grade tools
- Distributed training across clusters

### Production Deployment

- **TF Serving**: High-performance serving
- **TFLite**: Mobile and embedded devices
- **TF.js**: Browser and Node.js
- **TensorRT**: NVIDIA GPU optimization

## JAX

### Strengths

- Automatic differentiation (grad, vmap, jit)
- Just-in-time compilation (XLA)
- Functional programming paradigm
- Excellent for research and high-performance computing
- NumPy-like API

### When to Use

- High-performance research
- Scientific computing
- When you need JIT compilation
- Functional programming approach preferred
- Custom gradients and transformations

### Key Concepts

```python
# Auto-differentiation
from jax import grad
gradient_fn = grad(loss_fn)

# JIT compilation
from jax import jit
fast_fn = jit(slow_fn)

# Vectorization
from jax import vmap
batched_fn = vmap(single_fn)
```

## scikit-learn

### Strengths

- Simple, consistent API
- Comprehensive traditional ML algorithms
- Excellent documentation
- Preprocessing and feature engineering tools
- Model selection utilities

### When to Use

- Traditional ML (not deep learning)
- Tabular data
- When interpretability matters
- Quick prototyping with standard algorithms
- Small to medium datasets

### Key Algorithms

- Classification: SVM, Random Forest, Gradient Boosting
- Regression: Linear, Ridge, Lasso, Elastic Net
- Clustering: K-Means, DBSCAN, Hierarchical
- Dimensionality Reduction: PCA, t-SNE, UMAP

## Keras

### Strengths

- High-level, user-friendly API
- Fast prototyping
- Multi-backend support (TensorFlow, JAX)
- Sequential and Functional APIs

### When to Use

- Beginners learning deep learning
- Rapid prototyping
- Standard architectures
- When simplicity is priority

## Framework Selection Decision Tree

```
Need production deployment?
├─ Mobile/Edge → TensorFlow (TFLite)
├─ Web browser → TensorFlow.js
├─ High-scale serving → TensorFlow Serving or TorchServe
└─ General production → TensorFlow or PyTorch

Need research flexibility?
├─ Deep learning → PyTorch
├─ Functional programming → JAX
└─ Traditional ML → scikit-learn

Need high performance?
├─ GPU-optimized → JAX, PyTorch
├─ Distributed → TensorFlow, PyTorch
└─ CPU-optimized → scikit-learn

Beginner?
├─ Deep learning → Keras
├─ Traditional ML → scikit-learn
└─ Research → PyTorch tutorials
```

## Migration Considerations

### From TensorFlow to PyTorch

- Models can be converted via ONNX
- API philosophy is different (eager vs. graph)
- TensorBoard works with PyTorch

### From PyTorch to TensorFlow

- ONNX conversion available
- SavedModel format for serving
- May need to rewrite custom layers

## Best Practices

1. **Start Simple**: Use high-level APIs (Keras) first
2. **Profile Before Optimizing**: Measure bottlenecks
3. **Version Control**: Track model architecture and weights
4. **Reproducibility**: Set random seeds, save configs
5. **Mixed Precision**: Use FP16 for faster training (both frameworks support)

## Common Patterns

### PyTorch Training Loop

```python
model.train()
for epoch in range(num_epochs):
    for batch in dataloader:
        optimizer.zero_grad()
        outputs = model(batch['input'])
        loss = criterion(outputs, batch['target'])
        loss.backward()
        optimizer.step()
```

### TensorFlow Training Loop

```python
model.compile(optimizer='adam', loss='mse')
model.fit(train_dataset,
          validation_data=val_dataset,
          epochs=num_epochs,
          callbacks=[checkpoint, tensorboard])
```

### JAX Training Step

```python
@jit
def train_step(params, batch):
    def loss_fn(params):
        preds = model.apply(params, batch['x'])
        return jnp.mean((preds - batch['y'])**2)

    loss, grads = value_and_grad(loss_fn)(params)
    return loss, grads
```
