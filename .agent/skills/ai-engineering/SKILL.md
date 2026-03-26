---
name: ai-engineering
description: AI Engineering principles and decision-making for ML, DL, RL, and DRL. Framework selection, model architecture, training patterns, evaluation strategies, and deployment. Suitable from beginner to expert level. Use when working with machine learning, deep learning, reinforcement learning, model training, AI deployment, or MLOps tasks.
---

# AI Engineering

> Systematic approach to building AI systems from research to production.

## 1. Framework Selection

Choose the right framework based on your needs:

| Need                       | Framework       | Why                                        |
| -------------------------- | --------------- | ------------------------------------------ |
| **Research & prototyping** | PyTorch         | Dynamic graphs, pythonic, easy debugging   |
| **Production at scale**    | TensorFlow      | Mature ecosystem, TF Serving, TFLite       |
| **High performance**       | JAX             | JIT compilation, functional programming    |
| **Traditional ML**         | scikit-learn    | Simple API, comprehensive algorithms       |
| **Quick start**            | Keras           | High-level, beginner-friendly              |
| **Mobile/Edge**            | TensorFlow Lite | Optimized for resource-constrained devices |

**For detailed framework comparison, see [frameworks.md](references/frameworks.md)**

---

## 2. ML vs DL Decision

```
Dataset size < 10k rows?
├─ Yes → Traditional ML (scikit-learn)
└─ No → Consider deep learning

Tabular data?
├─ Yes → XGBoost, LightGBM, CatBoost
└─ No (images, text, audio) → Deep learning

Need interpretability?
├─ Yes → Decision trees, linear models
└─ No → Deep learning acceptable

Computational resources limited?
├─ Yes → Traditional ML or small neural networks
└─ No → Large deep learning models
```

---

## 3. Architecture Selection

| Data Type     | Task                      | Recommended Architecture                |
| ------------- | ------------------------- | --------------------------------------- |
| **Images**    | Classification            | ResNet, EffNet, ViT                     |
|               | Object Detection          | YOLOv8 (speed), Faster R-CNN (accuracy) |
|               | Segmentation              | U-Net, Mask R-CNN                       |
| **Text**      | Classification            | BERT, RoBERTa                           |
|               | Generation                | GPT, T5, BART                           |
|               | Translation               | T5, MarianMT                            |
| **Sequences** | Time Series               | LSTM, Temporal CNN, Transformer         |
|               | Speech                    | Wav2Vec 2.0, Whisper                    |
| **Tabular**   | Classification/Regression | XGBoost, LightGBM, Neural Networks      |

**For architecture details and variants, see [architectures.md](references/architectures.md)**

---

## 4. Training Workflow

### Standard Training Pipeline

1. **Data Preparation**
    - Load and explore data
    - Split: 70% train, 15% val, 15% test
    - Normalize/standardize
    - Augmentation (if needed)

2. **Model Selection**
    - Start simple (baseline model)
    - Choose architecture from table above
    - Consider transfer learning

3. **Training**
    - Initialize with good hyperparameters
    - Monitor train + validation metrics
    - Use callbacks (early stopping, lr scheduling)

4. **Evaluation**
    - Test set performance
    - Error analysis
    - Compare to baseline

5. **Optimization** (if needed)
    - Hyperparameter tuning
    - Architecture search
    - Ensemble methods

### Hyperparameter Tuning Strategy

| Method                    | When to Use                               |
| ------------------------- | ----------------------------------------- |
| **Grid Search**           | Small search space (< 10 combinations)    |
| **Random Search**         | Medium space (< 100 combinations)         |
| **Bayesian Optimization** | Expensive training, continuous parameters |
| **Population-based**      | Very large models, parallel resources     |

---

## 5. Reinforcement Learning

### RL vs Supervised Learning

Use RL when:

- No labeled data, only reward signals
- Sequential decision making
- Agent-environment interaction
- Need to learn optimal policy

### Algorithm Selection

| Action Space   | Sample Efficiency | Algorithm              |
| -------------- | ----------------- | ---------------------- |
| Discrete       | Low priority      | DQN, Rainbow           |
| Discrete       | High priority     | SAC (discrete version) |
| Continuous     | Low priority      | PPO                    |
| Continuous     | High priority     | SAC, TD3               |
| Need stability | -                 | PPO (most stable)      |

**For RL/DRL implementation details, see [rl-drl.md](references/rl-drl.md)**

---

## 6. Model Evaluation

### Metrics by Task Type

| Task                      | Primary Metrics    | When to Use Others                                                  |
| ------------------------- | ------------------ | ------------------------------------------------------------------- |
| **Binary Classification** | F1, AUC-ROC        | Precision (false positives matter), Recall (false negatives matter) |
| **Multi-class**           | Macro F1, Accuracy | Per-class F1 (imbalanced), Confusion matrix (error analysis)        |
| **Regression**            | MSE, MAE           | R² (goodness of fit), MAPE (percentage error)                       |
| **Object Detection**      | mAP                | IoU thresholds, per-class AP                                        |
| **RL**                    | Cumulative reward  | Episode length, success rate                                        |

### Validation Strategy

```
Data size < 1000 samples?
├─ Yes → K-fold cross-validation (k=5 or 10)
└─ No → Single train/val/test split

Time series data?
├─ Yes → Time-based splits (no shuffle!)
└─ No → Random or stratified split

Imbalanced classes?
├─ Yes → Stratified split
└─ No → Random split
```

---

## 7. Deployment Decision Tree

```
Where will model run?
├─ Cloud → API serving (TF Serving, TorchServe)
├─ Edge/Mobile → Model compression + TFLite/ONNX
├─ Browser → TensorFlow.js
└─ Batch → Scheduled jobs

Latency requirements?
├─ Real-time (< 100ms) → Optimize model, use caching
├─ Interactive (< 1s) → Standard serving
└─ Batch (minutes/hours) → Batch processing

Scale?
├─ High traffic → Kubernetes + auto-scaling
├─ Medium → Cloud Run, Lambda
└─ Low → Simple API server
```

### Model Optimization

Before deployment, consider:

- **Quantization**: 4x smaller, 2x faster (INT8)
- **Pruning**: Remove redundant weights
- **Distillation**: Smaller student model
- **ONNX**: Cross-framework deployment

**For MLOps and production deployment details, see [mlops.md](references/mlops.md)**

---

## 8. Common Decision Points

### Transfer Learning vs Train from Scratch

Use transfer learning when:

- ✅ Similar domain (ImageNet → custom images)
- ✅ Limited data (< 10k samples)
- ✅ Limited compute

Train from scratch when:

- ❌ Very different domain
- ❌ Huge dataset (millions of samples)
- ❌ Need maximum performance

### Data Augmentation

**Vision:**

- Always: Resize/crop, normalization
- Usually: Random flip, color jitter
- Sometimes: Mixup, CutMix, AutoAugment

**NLP:**

- Back-translation
- Synonym replacement
- Contextual word embeddings

**Time Series:**

- Window slicing
- Noise injection
- Time warping

---

## 9. Quick Start Checklist

- [ ] Define problem (classification, regression, generation, RL?)
- [ ] Choose framework (see section 1)
- [ ] Prepare data (clean, split, augment)
- [ ] Select baseline architecture (see section 3)
- [ ] Train with default hyperparameters
- [ ] Evaluate on validation set
- [ ] Analyze errors
- [ ] Iterate (tune, improve)
- [ ] Evaluate on test set
- [ ] Plan deployment (see section 7)

---

## 10. Common Pitfalls

| Problem                 | Solution                                                         |
| ----------------------- | ---------------------------------------------------------------- |
| **Overfitting**         | Regularization (dropout, L2), more data, simpler model           |
| **Underfitting**        | Larger model, more features, less regularization                 |
| **Slow training**       | Larger batch size, better optimizer (Adam), learning rate tuning |
| **Unstable training**   | Lower learning rate, gradient clipping, batch normalization      |
| **Poor generalization** | Data augmentation, cross-validation, domain adaptation           |
| **Class imbalance**     | Class weights, resampling, proper metrics (F1, not accuracy)     |

---

> **Philosophy:** Start simple, measure everything, iterate based on data. The best model is the one that solves the problem with minimum complexity.

## References

- [frameworks.md](references/frameworks.md) - Deep dive into PyTorch, TensorFlow, JAX, scikit-learn
- [architectures.md](references/architectures.md) - CNN, RNN, Transformer, GAN details
- [rl-drl.md](references/rl-drl.md) - Reinforcement learning algorithms and implementation
- [mlops.md](references/mlops.md) - Production ML systems and deployment
