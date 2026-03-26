# MLOps - Production ML Systems

## Model Lifecycle

### 1. Experimentation & Development

**Goal**: Find best model for the problem

**Activities:**

- Data exploration and analysis
- Feature engineering experiments
- Model architecture search
- Hyperparameter tuning
- Track experiments

**Tools:**

- **Jupyter/Colab**: Interactive development
- **MLflow**: Experiment tracking
- **Weights & Biases**: Experiment tracking + visualization
- **Neptune.ai**: Experiment tracking
- **TensorBoard**: Visualization

### 2. Training Pipeline

**Goal**: Reproducible, automated training

**Best Practices:**

- Version control code (Git)
- Version control data (DVC, Pachyderm)
- Config-driven training
- Automated hyperparameter search
- Distributed training for large models

**Tools:**

- **DVC**: Data version control
- **Kubeflow**: ML workflows on Kubernetes
- **Apache Airflow**: Workflow orchestration
- **Prefect**: Modern workflow orchestration

### 3. Model Validation

**Goal**: Ensure model quality before deployment

**Validation Steps:**

- Performance metrics on test set
- Cross-validation
- Error analysis
- Fairness and bias testing
- A/B testing framework

**Tools:**

- **Great Expectations**: Data validation
- **Evidently AI**: ML monitoring
- **Alibi Detect**: Drift detection

### 4. Model Deployment

**Goal**: Serve model in production

**Deployment Patterns:**

- **REST API**: Flask, FastAPI
- **gRPC**: High-performance serving
- **Batch**: Scheduled predictions
- **Edge**: On-device inference
- **Streaming**: Real-time data

**Serving Tools:**

- **TensorFlow Serving**: TF models
- **TorchServe**: PyTorch models
- **KServe**: Kubernetes-native
- **BentoML**: Framework-agnostic
- **Seldon Core**: ML deployment on K8s

### 5. Monitoring

**Goal**: Track model performance in production

**What to Monitor:**

- Model performance (accuracy, latency)
- Data drift (input distribution changes)
- Concept drift (target distribution changes)
- Infrastructure metrics (CPU, memory, GPU)
- Business metrics (conversion, revenue impact)

**Tools:**

- **Prometheus + Grafana**: Metrics and dashboards
- **Evidently**: ML-specific monitoring
- **Arize**: ML observability
- **Fiddler**: Model monitoring

### 6. Retraining

**Goal**: Keep model up-to-date

**Triggers:**

- Performance degradation
- Data drift detected
- Scheduled (weekly/monthly)
- New data available

**Automation:**

- Automated retraining pipeline
- Automated validation
- Canary deployment
- Rollback capability

## Model Registry

**Purpose**: Centralized model storage and versioning

**Features:**

- Model versioning
- Metadata (metrics, parameters)
- Lineage tracking
- Stage management (staging, production)

**Tools:**

- **MLflow Model Registry**
- **AWS SageMaker Model Registry**
- **Azure ML Model Registry**
- **Neptune Model Registry**

## CI/CD for ML

### Continuous Integration

- Unit tests for preprocessing
- Model tests (smoke tests)
- Data validation tests
- Integration tests

### Continuous Deployment

- Canary deployment (gradual rollout)
- Blue-green deployment
- Shadow deployment (parallel testing)
- Feature flags

### Tools

- **GitHub Actions**: CI/CD automation
- **GitLab CI**: Integrated CI/CD
- **Jenkins**: Traditional CI/CD
- **ArgoCD**: Kubernetes deployments

## Feature Store

**Purpose**: Centralized feature management

**Benefits:**

- Feature reuse across teams
- Consistency (training/serving)
- Real-time feature serving
- Feature versioning

**Tools:**

- **Feast**: Open-source feature store
- **Tecton**: Enterprise feature platform
- **Hopsworks**: ML data platform
- **AWS Feature Store**

## Data Versioning

**Why Version Data:**

- Reproducibility
- Debugging
- Compliance
- Experiment tracking

**Tools:**

- **DVC**: Git for data
- **Pachyderm**: Data pipelines
- **LakeFS**: Data lake versioning
- **Delta Lake**: ACID for data lakes

## Model Optimization for Production

### Quantization

```python
# PyTorch quantization
import torch.quantization
model_int8 = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)
```

**Benefits:**

- 4x smaller model size
- 2-4x faster inference
- Minimal accuracy loss (<1%)

### ONNX Export

```python
# Export to ONNX (cross-framework)
torch.onnx.export(model, dummy_input, "model.onnx")
```

**Benefits:**

- Framework-agnostic deployment
- Hardware optimization (TensorRT, OpenVINO)

### Model Compression Techniques

- **Pruning**: Remove low-importance weights
- **Distillation**: Train smaller student model
- **Low-rank factorization**: Decompose weight matrices

## Infrastructure Patterns

### Serverless ML

- **AWS Lambda**: Event-driven inference
- **Google Cloud Functions**: Serverless functions
- **Azure Functions**: Serverless compute

**Use When:**

- Sporadic requests
- Auto-scaling needed
- Cost optimization important

### Kubernetes-based

- **Kubeflow**: ML platform on K8s
- **Seldon Core**: Model deployment
- **KServe**: Serverless inference

**Use When:**

- High-scale serving
- Multi-model deployment
- Need orchestration

### Managed ML Platforms

- **AWS SageMaker**: End-to-end ML
- **Azure ML**: Microsoft's ML platform
- **Google Vertex AI**: Unified ML platform

**Use When:**

- Prefer managed services
- Need enterprise features
- Quick time-to-market

## Security Best Practices

- **Model Protection**: Encrypt models at rest
- **API Security**: Authentication (API keys, OAuth)
- **Data Privacy**: PII handling, GDPR compliance
- **Access Control**: Role-based access (RBAC)
- **Audit Logging**: Track model access and predictions

## Cost Optimization

- **Auto-scaling**: Scale down during low traffic
- **Spot instances**: Use preemptible VMs for training
- **Model optimization**: Reduce inference cost
- **Batch predictions**: Group requests
- **Caching**: Cache frequent predictions

## MLOps Maturity Model

### Level 0: Manual Process

- Manual training, deployment
- No automation
- Notebooks in production

### Level 1: ML Pipeline Automation

- Automated training pipeline
- Experiment tracking
- Basic deployment

### Level 2: CI/CD

- Automated testing
- Continuous deployment
- Monitoring

### Level 3: Full MLOps

- Automated retraining
- Feature stores
- Model governance
- Production ML platform

## Common Anti-Patterns

❌ **Training-serving skew**: Different preprocessing in training vs. serving
✅ **Solution**: Use feature stores, unify preprocessing

❌ **Hidden technical debt**: Tangled dependencies, glue code
✅ **Solution**: Modular design, clear interfaces

❌ **No monitoring**: Deploy and forget
✅ **Solution**: Comprehensive monitoring, alerts

❌ **Manual processes**: Manual retraining, deployment
✅ **Solution**: Automate everything

## Tools Comparison Matrix

| Category                | Open Source            | Enterprise                |
| ----------------------- | ---------------------- | ------------------------- |
| **Experiment Tracking** | MLflow, TensorBoard    | Weights & Biases, Neptune |
| **Serving**             | TorchServe, TF Serving | SageMaker, Vertex AI      |
| **Monitoring**          | Prometheus, Evidently  | Arize, Fiddler            |
| **Orchestration**       | Airflow, Prefect       | AWS Step Functions        |
| **Feature Store**       | Feast                  | Tecton, Hopsworks         |

## Getting Started Checklist

- [ ] Set up experiment tracking (MLflow or W&B)
- [ ] Version control data (DVC)
- [ ] Create training pipeline (config-driven)
- [ ] Implement model registry
- [ ] Set up model serving (REST API)
- [ ] Add monitoring (Prometheus + Grafana)
- [ ] Implement CI/CD
- [ ] Plan retraining strategy
- [ ] Document model lifecycle
