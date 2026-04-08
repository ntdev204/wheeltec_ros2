# Deep Learning Architectures - Details

## Convolutional Neural Networks (CNN)

### Core Concepts

- **Convolution**: Feature extraction via learned filters
- **Pooling**: Spatial downsampling
- **Fully Connected**: Final classification/regression

### Popular Architectures

#### ResNet (Residual Networks)

- **Innovation**: Skip connections (x + F(x))
- **Solves**: Vanishing gradient problem
- **Variants**: ResNet-18, 34, 50, 101, 152
- **Use**: Image classification, feature extraction

####EfficientNet

- **Innovation**: Compound scaling (depth, width, resolution)
- **Strengths**: State-of-art accuracy/efficiency trade-off
- **Variants**: B0 to B7
- **Use**: Resource-constrained environments

#### Vision Transformer (ViT)

- **Innovation**: Apply transformers to image patches
- **Strengths**: No inductive biases, scales well with data
- **Use**: Large-scale image classification

### Object Detection

#### YOLO (You Only Look Once)

- **Type**: Single-stage detector
- **Speed**: Real-time (30+ FPS)
- **Variants**: YOLOv3, v4, v5, v7, v8
- **Use**: Real-time object detection

#### Faster R-CNN

- **Type**: Two-stage detector (region proposal + classification)
- **Accuracy**: Higher than YOLO
- **Speed**: Slower (~5-10 FPS)
- **Use**: High-accuracy detection

### Segmentation

#### U-Net

- **Architecture**: Encoder-decoder with skip connections
- **Use**: Medical image segmentation, semantic segmentation

#### Mask R-CNN

- **Extension**: Faster R-CNN + instance segmentation
- **Output**: Bounding boxes + pixel-level masks

## Recurrent Neural Networks (RNN)

### LSTM (Long Short-Term Memory)

- **Solves**: Vanishing gradient in RNNs
- **Components**: Forget gate, input gate, output gate
- **Use**: Sequence modeling, time series

### GRU (Gated Recurrent Unit)

- **Simplified**: Fewer gates than LSTM
- **Performance**: Similar to LSTM, faster training
- **Use**: When simpler model suffices

## Transformers

### Core Mechanism: Self-Attention

```
Attention(Q, K, V) = softmax(QK^T / √d_k)V
- Q: Query
- K: Key
- V: Value
```

### BERT (Bidirectional Encoder)

- **Training**: Masked language modeling
- **Architecture**: Encoder-only transformer
- **Use**: Text classification, NER, QA

### GPT (Generative Pre-trained Transformer)

- **Training**: Autoregressive language modeling
- **Architecture**: Decoder-only transformer
- **Use**: Text generation, completion

### T5 (Text-to-Text Transfer Transformer)

- **Approach**: All tasks as text-to-text
- **Versatility**: Translation, QA, summarization

## Generative Models

### GAN (Generative Adversarial Network)

- **Components**: Generator + Discriminator
- **Training**: Adversarial loss
- **Variants**:
    - **DCGAN**: Deep convolutional GAN
    - **StyleGAN**: Style-based generator
    - **Pix2Pix**: Image-to-image translation
    - **CycleGAN**: Unpaired image translation

### VAE (Variational Autoencoder)

- **Approach**: Probabilistic encoder-decoder
- **Latent Space**: Continuous, structured
- **Use**: Generation, semi-supervised learning

### Diffusion Models

- **Examples**: Stable Diffusion, DALL-E 2
- **Process**: Iterative denoising
- **Use**: High-quality image generation

## Architecture Selection Guide

| Task                    | Recommended Architecture                |
| ----------------------- | --------------------------------------- |
| Image Classification    | EfficientNet, ResNet, ViT               |
| Object Detection        | YOLOv8 (speed), Faster R-CNN (accuracy) |
| Instance Segmentation   | Mask R-CNN                              |
| Semantic Segmentation   | U-Net, DeepLabv3+                       |
| Text Classification     | BERT, RoBERTa, DistilBERT               |
| Text Generation         | GPT-3, T5, BART                         |
| Machine Translation     | T5, mBART, MarianMT                     |
| Question Answering      | BERT, RoBERTa, ELECTRA                  |
| Image Generation        | StyleGAN, Stable Diffusion              |
| Time Series Forecasting | LSTM, Temporal CNN, Transformer         |
| Speech Recognition      | Wav2Vec 2.0, Whisper                    |

## Transfer Learning

### Pre-trained Models

**Computer Vision:**

- ImageNet pre-trained: ResNet, EfficientNet
- COCO pre-trained: YOLO, Faster R-CNN

**NLP:**

- Hugging Face Hub: BERT, GPT, T5, etc.

### Fine-tuning Strategies

1. **Feature Extraction**: Freeze base, train head
2. **Fine-tune All**: Unfreeze all, low learning rate
3. **Layer-wise**: Gradually unfreeze layers
4. **Adapter**: Add small trainable modules (PEFT)

## Model Compression

| Technique                      | Description                    | Trade-off                 |
| ------------------------------ | ------------------------------ | ------------------------- |
| **Quantization**               | Reduce precision (FP32 → INT8) | Speed ↑, Accuracy ↓       |
| **Pruning**                    | Remove low-importance weights  | Size ↓, Accuracy ↓        |
| **Distillation**               | Train smaller student model    | Size ↓, Train time ↑      |
| **Neural Architecture Search** | Automated architecture design  | Accuracy ↑, Search cost ↑ |

## Common Patterns

### Data Augmentation (Vision)

- Random crop, flip, rotation
- Color jitter, normalization
- Mixup, CutMix
- AutoAugment

### Data Augmentation (NLP)

- Back-translation
- Synonym replacement
- Random insertion/deletion
- Paraphrasing

### Regularization

- Dropout (0.2-0.5)
- Batch Normalization
- Layer Normalization (transformers)
- Weight Decay (L2 regularization)
