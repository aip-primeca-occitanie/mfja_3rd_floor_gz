#!/usr/bin/env python3

import argparse
import json
import math
import random
import re
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


DEFAULT_SPLITS_DIR = Path('/home/tiago/room315_local_training/splits')
DEFAULT_OUTPUT_DIR = Path('/home/tiago/room315_local_training/checkpoints/v0')
DEFAULT_DATASET_ROOT = Path('/home/tiago/room315_payload_expanded_160_merged_final')
TEXT_PAD = '<pad>'
TEXT_UNK = '<unk>'
SPEED_INDEX = 19


def _pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def _iter_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f'{path}:{line_number}: invalid JSONL row: {exc}') from exc
            if not isinstance(parsed, dict):
                raise ValueError(f'{path}:{line_number}: JSONL row must be an object')
            rows.append(parsed)
    return rows


def _require_torch():
    try:
        import torch
    except ImportError as exc:
        raise SystemExit(
            'PyTorch is required for local neural training but is not installed.\n'
            'Install it first, then rerun this command. For this machine, start with:\n'
            '  python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cu128\n'
            'Or choose the current command from https://pytorch.org/get-started/locally/.'
        ) from exc
    return torch


def _load_manifest(splits_dir: Path) -> dict[str, Any]:
    manifest_path = splits_dir / 'split_manifest.json'
    if not manifest_path.exists():
        return {}
    with manifest_path.open('r', encoding='utf-8') as stream:
        parsed = json.load(stream)
    return parsed if isinstance(parsed, dict) else {}


def _resolve_dataset_root(dataset_root: Path | None, splits_dir: Path) -> Path:
    if dataset_root is not None:
        return dataset_root.expanduser().resolve()
    manifest = _load_manifest(splits_dir)
    root = str(manifest.get('dataset_root') or '').strip()
    if root:
        return Path(root).expanduser().resolve()
    return DEFAULT_DATASET_ROOT.expanduser().resolve()


def tokenize(text: Any) -> list[str]:
    return re.findall(r'[A-Za-z0-9_]+', str(text or '').lower())


def _language(row: dict[str, Any]) -> str:
    model_input = row.get('model_input')
    if isinstance(model_input, dict) and model_input.get('language'):
        return str(model_input.get('language') or '')
    return str(row.get('task') or row.get('generated_language') or '')


def build_vocab(
    rows: list[dict[str, Any]],
    *,
    max_vocab_size: int = 2048,
    min_freq: int = 1,
) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for row in rows:
        counts.update(tokenize(_language(row)))
    tokens = [
        token
        for token, count in sorted(counts.items(), key=lambda item: (-item[1], item[0]))
        if count >= min_freq
    ]
    vocab = {TEXT_PAD: 0, TEXT_UNK: 1}
    for token in tokens[:max(0, max_vocab_size - len(vocab))]:
        vocab[token] = len(vocab)
    return vocab


def encode_text(text: Any, vocab: dict[str, int], max_tokens: int) -> np.ndarray:
    token_ids = [vocab.get(token, vocab[TEXT_UNK]) for token in tokenize(text)]
    token_ids = token_ids[:max_tokens]
    if len(token_ids) < max_tokens:
        token_ids.extend([vocab[TEXT_PAD]] * (max_tokens - len(token_ids)))
    return np.asarray(token_ids, dtype=np.int64)


def _flatten(prefix: str, value: Any, output: dict[str, Any]) -> None:
    if isinstance(value, dict):
        for key in sorted(value):
            child_prefix = f'{prefix}.{key}' if prefix else str(key)
            _flatten(child_prefix, value[key], output)
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            child_prefix = f'{prefix}.{index}' if prefix else str(index)
            _flatten(child_prefix, child, output)
        return
    output[prefix] = value


def _to_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _model_input_state(row: dict[str, Any]) -> dict[str, Any]:
    model_input = row.get('model_input')
    if not isinstance(model_input, dict):
        return {}
    state = {}
    _flatten('observable_state', model_input.get('observable_state', {}), state)
    _flatten('last_command', model_input.get('last_command', {}), state)
    return state


class StateVectorizer:
    def __init__(
        self,
        numeric_keys: list[str],
        categorical_values: dict[str, list[str]],
    ) -> None:
        self.numeric_keys = list(numeric_keys)
        self.categorical_values = {
            key: list(values)
            for key, values in categorical_values.items()
        }

    @classmethod
    def fit(cls, rows: list[dict[str, Any]]) -> 'StateVectorizer':
        numeric_keys: set[str] = set()
        categorical_values: dict[str, set[str]] = {}
        for row in rows:
            for key, value in _model_input_state(row).items():
                if _to_float(value) is None:
                    categorical_values.setdefault(key, set()).add(str(value).strip().lower())
                else:
                    numeric_keys.add(key)
        return cls(
            sorted(numeric_keys),
            {key: sorted(values) for key, values in sorted(categorical_values.items())},
        )

    @property
    def dim(self) -> int:
        return len(self.numeric_keys) + sum(len(values) for values in self.categorical_values.values())

    def transform(self, row: dict[str, Any]) -> np.ndarray:
        values = _model_input_state(row)
        vector: list[float] = []
        for key in self.numeric_keys:
            vector.append(_to_float(values.get(key)) or 0.0)
        for key, allowed_values in self.categorical_values.items():
            raw = str(values.get(key, '')).strip().lower()
            vector.extend(1.0 if raw == allowed else 0.0 for allowed in allowed_values)
        return np.asarray(vector, dtype=np.float32)

    def to_json(self) -> dict[str, Any]:
        return {
            'numeric_keys': self.numeric_keys,
            'categorical_values': self.categorical_values,
            'dim': self.dim,
        }


def _image_refs(row: dict[str, Any]) -> dict[str, str]:
    model_input = row.get('model_input')
    if isinstance(model_input, dict) and isinstance(model_input.get('overhead_images'), dict):
        return {
            str(key): str(value)
            for key, value in model_input['overhead_images'].items()
            if value
        }
    refs = row.get('image_frame_refs')
    if isinstance(refs, dict):
        return {str(key): str(value) for key, value in refs.items() if value}
    return {
        key.removeprefix('observation.images.'): str(value)
        for key, value in row.items()
        if key.startswith('observation.images.') and value
    }


def _resolve_image_path(dataset_root: Path, image_ref: str) -> Path:
    path = Path(str(image_ref)).expanduser()
    return path if path.is_absolute() else dataset_root / path


def load_image_tensor(
    dataset_root: Path,
    image_ref: str,
    *,
    width: int,
    height: int,
) -> np.ndarray:
    if not image_ref:
        return np.zeros((3, height, width), dtype=np.float32)
    image_path = _resolve_image_path(dataset_root, image_ref)
    if not image_path.exists():
        return np.zeros((3, height, width), dtype=np.float32)
    with Image.open(image_path) as image:
        rgb = image.convert('RGB').resize((width, height), Image.BILINEAR)
        array = np.asarray(rgb, dtype=np.float32) / 255.0
    return np.transpose(array, (2, 0, 1))


def load_paired_images(
    row: dict[str, Any],
    dataset_root: Path,
    *,
    width: int,
    height: int,
) -> np.ndarray:
    refs = _image_refs(row)
    left = load_image_tensor(dataset_root, refs.get('left_rail_rgb', ''), width=width, height=height)
    right = load_image_tensor(dataset_root, refs.get('right_rail_rgb', ''), width=width, height=height)
    return np.concatenate([left, right], axis=0).astype(np.float32)


def action_vector(row: dict[str, Any]) -> np.ndarray:
    raw = row.get('action_vector')
    if not isinstance(raw, list):
        raise ValueError(f'row for episode {row.get("episode_id", "")!r} is missing action_vector')
    return np.asarray([float(value) for value in raw], dtype=np.float32)


def target_stats(rows: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
    targets = np.stack([action_vector(row) for row in rows], axis=0).astype(np.float32)
    mean = targets.mean(axis=0)
    std = targets.std(axis=0)
    std[std < 1e-6] = 1.0
    return mean.astype(np.float32), std.astype(np.float32)


class Room315EventDataset:
    def __init__(
        self,
        rows: list[dict[str, Any]],
        *,
        dataset_root: Path,
        vocab: dict[str, int],
        state_vectorizer: StateVectorizer,
        target_mean: np.ndarray,
        target_std: np.ndarray,
        max_tokens: int,
        image_width: int,
        image_height: int,
        torch_module: Any,
    ) -> None:
        self.rows = rows
        self.dataset_root = dataset_root
        self.vocab = vocab
        self.state_vectorizer = state_vectorizer
        self.target_mean = target_mean
        self.target_std = target_std
        self.max_tokens = max_tokens
        self.image_width = image_width
        self.image_height = image_height
        self.torch = torch_module

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, Any]:
        row = self.rows[index]
        target = action_vector(row)
        normalized_target = (target - self.target_mean) / self.target_std
        return {
            'text': self.torch.as_tensor(
                encode_text(_language(row), self.vocab, self.max_tokens),
                dtype=self.torch.long,
            ),
            'state': self.torch.as_tensor(
                self.state_vectorizer.transform(row),
                dtype=self.torch.float32,
            ),
            'image': self.torch.as_tensor(
                load_paired_images(
                    row,
                    self.dataset_root,
                    width=self.image_width,
                    height=self.image_height,
                ),
                dtype=self.torch.float32,
            ),
            'target': self.torch.as_tensor(normalized_target, dtype=self.torch.float32),
            'raw_target': self.torch.as_tensor(target, dtype=self.torch.float32),
        }


def _build_model(torch_module: Any, *, vocab_size: int, state_dim: int, output_dim: int):
    nn = torch_module.nn

    class Room315LocalVlaModel(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.embedding = nn.Embedding(vocab_size, 96, padding_idx=0)
            self.image_encoder = nn.Sequential(
                nn.Conv2d(6, 24, kernel_size=5, stride=2, padding=2),
                nn.ReLU(inplace=True),
                nn.Conv2d(24, 48, kernel_size=3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.Conv2d(48, 96, kernel_size=3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((1, 1)),
                nn.Flatten(),
            )
            self.state_encoder = nn.Sequential(
                nn.Linear(max(1, state_dim), 96),
                nn.ReLU(inplace=True),
                nn.Linear(96, 96),
                nn.ReLU(inplace=True),
            )
            self.head = nn.Sequential(
                nn.Linear(96 + 96 + 96, 192),
                nn.ReLU(inplace=True),
                nn.Dropout(0.1),
                nn.Linear(192, output_dim),
            )

        def forward(self, text, state, image):
            embedded = self.embedding(text)
            mask = (text != 0).unsqueeze(-1).float()
            text_features = (embedded * mask).sum(dim=1) / mask.sum(dim=1).clamp_min(1.0)
            image_features = self.image_encoder(image)
            state_features = self.state_encoder(state)
            return self.head(torch_module.cat([text_features, image_features, state_features], dim=1))

    return Room315LocalVlaModel()


def _choose_device(torch_module: Any, requested: str) -> str:
    requested = str(requested or 'auto').strip().lower()
    if requested == 'auto':
        return 'cuda' if torch_module.cuda.is_available() else 'cpu'
    if requested == 'cuda' and not torch_module.cuda.is_available():
        raise SystemExit('CUDA was requested, but torch.cuda.is_available() is false.')
    return requested


def _set_seed(torch_module: Any, seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch_module.manual_seed(seed)
    if torch_module.cuda.is_available():
        torch_module.cuda.manual_seed_all(seed)


def _row_limit(rows: list[dict[str, Any]], limit: int | None) -> list[dict[str, Any]]:
    if limit is None or limit <= 0:
        return rows
    return rows[:limit]


def _evaluate(
    torch_module: Any,
    model: Any,
    loader: Any,
    *,
    device: str,
    target_mean: np.ndarray,
    target_std: np.ndarray,
) -> dict[str, float]:
    model.eval()
    loss_fn = torch_module.nn.SmoothL1Loss(reduction='sum')
    mean_tensor = torch_module.as_tensor(target_mean, dtype=torch_module.float32, device=device)
    std_tensor = torch_module.as_tensor(target_std, dtype=torch_module.float32, device=device)
    rows = 0
    loss_total = 0.0
    mae_total = 0.0
    discrete_correct = 0
    discrete_total = 0
    exact_count = 0
    primitive_correct = 0
    side_correct = 0
    target_correct = 0
    speed_error_total = 0.0
    with torch_module.no_grad():
        for batch in loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            raw_target = batch['raw_target'].to(device)
            pred_norm = model(text, state, image)
            loss_total += float(loss_fn(pred_norm, target).item())
            pred = pred_norm * std_tensor + mean_tensor
            abs_error = (pred - raw_target).abs()
            batch_size = int(raw_target.shape[0])
            rows += batch_size
            mae_total += float(abs_error.sum().item())
            speed_error_total += float(abs_error[:, SPEED_INDEX].sum().item())

            discrete_indices = [idx for idx in range(raw_target.shape[1]) if idx != SPEED_INDEX]
            pred_discrete = pred[:, discrete_indices].round()
            true_discrete = raw_target[:, discrete_indices].round()
            matches = pred_discrete.eq(true_discrete)
            discrete_correct += int(matches.sum().item())
            discrete_total += int(matches.numel())
            speed_ok = abs_error[:, SPEED_INDEX] <= 0.005
            exact = matches.all(dim=1) & speed_ok
            exact_count += int(exact.sum().item())
            primitive_correct += int(pred[:, 0].round().eq(raw_target[:, 0].round()).sum().item())
            side_correct += int(pred[:, 1].round().eq(raw_target[:, 1].round()).sum().item())
            target_correct += int(pred[:, 21].round().eq(raw_target[:, 21].round()).sum().item())
    denom = max(1, rows)
    return {
        'loss': round(loss_total / denom, 6),
        'vector_mae': round(mae_total / max(1, rows * len(target_mean)), 6),
        'discrete_field_accuracy': round(discrete_correct / max(1, discrete_total), 6),
        'exact_action_accuracy': round(exact_count / denom, 6),
        'primitive_accuracy': round(primitive_correct / denom, 6),
        'side_accuracy': round(side_correct / denom, 6),
        'target_id_accuracy': round(target_correct / denom, 6),
        'speed_mae': round(speed_error_total / denom, 6),
    }


def _save_checkpoint(
    torch_module: Any,
    path: Path,
    *,
    model: Any,
    optimizer: Any,
    epoch: int,
    metrics: dict[str, Any],
    config: dict[str, Any],
) -> None:
    torch_module.save(
        {
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'metrics': metrics,
            'config': config,
        },
        path,
    )


def train_local(args: argparse.Namespace) -> dict[str, Any]:
    torch_module = _require_torch()
    splits_dir = args.splits_dir.expanduser().resolve()
    dataset_root = _resolve_dataset_root(args.dataset_root, splits_dir)
    train_rows = _row_limit(_iter_jsonl(splits_dir / args.train_file), args.limit_train_rows)
    val_rows = _row_limit(_iter_jsonl(splits_dir / args.val_file), args.limit_val_rows)
    if not train_rows:
        raise ValueError('train split is empty')
    if not val_rows:
        raise ValueError('validation split is empty')

    _set_seed(torch_module, args.seed)
    device = _choose_device(torch_module, args.device)
    vocab = build_vocab(
        train_rows,
        max_vocab_size=args.max_vocab_size,
        min_freq=args.min_token_frequency,
    )
    state_vectorizer = StateVectorizer.fit(train_rows)
    target_mean, target_std = target_stats(train_rows)
    output_dim = int(target_mean.shape[0])
    config = {
        'dataset_root': str(dataset_root),
        'splits_dir': str(splits_dir),
        'train_file': args.train_file,
        'val_file': args.val_file,
        'seed': args.seed,
        'device': device,
        'epochs': args.epochs,
        'batch_size': args.batch_size,
        'learning_rate': args.learning_rate,
        'weight_decay': args.weight_decay,
        'image_width': args.image_width,
        'image_height': args.image_height,
        'max_tokens': args.max_tokens,
        'max_vocab_size': args.max_vocab_size,
        'train_rows': len(train_rows),
        'val_rows': len(val_rows),
        'output_dim': output_dim,
        'state_dim': state_vectorizer.dim,
        'vocab_size': len(vocab),
    }

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'vocab.json').write_text(_pretty_json(vocab) + '\n', encoding='utf-8')
    (output_dir / 'state_vectorizer.json').write_text(
        _pretty_json(state_vectorizer.to_json()) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'target_stats.json').write_text(
        _pretty_json({
            'mean': target_mean.tolist(),
            'std': target_std.tolist(),
        }) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'training_config.json').write_text(_pretty_json(config) + '\n', encoding='utf-8')

    train_dataset = Room315EventDataset(
        train_rows,
        dataset_root=dataset_root,
        vocab=vocab,
        state_vectorizer=state_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
    )
    val_dataset = Room315EventDataset(
        val_rows,
        dataset_root=dataset_root,
        vocab=vocab,
        state_vectorizer=state_vectorizer,
        target_mean=target_mean,
        target_std=target_std,
        max_tokens=args.max_tokens,
        image_width=args.image_width,
        image_height=args.image_height,
        torch_module=torch_module,
    )
    train_loader = torch_module.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )
    val_loader = torch_module.utils.data.DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device == 'cuda'),
    )

    model = _build_model(
        torch_module,
        vocab_size=len(vocab),
        state_dim=state_vectorizer.dim,
        output_dim=output_dim,
    ).to(device)
    optimizer = torch_module.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    loss_fn = torch_module.nn.SmoothL1Loss()
    history: list[dict[str, Any]] = []
    best_val_loss = float('inf')
    best_epoch = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        seen = 0
        for batch in train_loader:
            text = batch['text'].to(device)
            state = batch['state'].to(device)
            image = batch['image'].to(device)
            target = batch['target'].to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(text, state, image)
            loss = loss_fn(pred, target)
            loss.backward()
            optimizer.step()
            batch_size = int(target.shape[0])
            running_loss += float(loss.item()) * batch_size
            seen += batch_size
        train_loss = running_loss / max(1, seen)
        val_metrics = _evaluate(
            torch_module,
            model,
            val_loader,
            device=device,
            target_mean=target_mean,
            target_std=target_std,
        )
        epoch_metrics = {
            'epoch': epoch,
            'train_loss': round(train_loss, 6),
            **{f'val_{key}': value for key, value in val_metrics.items()},
        }
        history.append(epoch_metrics)
        print(_pretty_json(epoch_metrics), flush=True)
        _save_checkpoint(
            torch_module,
            output_dir / 'last.pt',
            model=model,
            optimizer=optimizer,
            epoch=epoch,
            metrics=epoch_metrics,
            config=config,
        )
        if val_metrics['loss'] < best_val_loss:
            best_val_loss = val_metrics['loss']
            best_epoch = epoch
            _save_checkpoint(
                torch_module,
                output_dir / 'best.pt',
                model=model,
                optimizer=optimizer,
                epoch=epoch,
                metrics=epoch_metrics,
                config=config,
            )

    summary = {
        'output_dir': str(output_dir),
        'best_checkpoint': str(output_dir / 'best.pt'),
        'last_checkpoint': str(output_dir / 'last.pt'),
        'best_epoch': best_epoch,
        'best_val_loss': best_val_loss,
        'history': history,
        'config': config,
    }
    (output_dir / 'metrics.json').write_text(_pretty_json(summary) + '\n', encoding='utf-8')
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Train a compact local Room 315 VLA policy from train/val JSONL splits.'
    )
    parser.add_argument('--splits-dir', type=Path, default=DEFAULT_SPLITS_DIR)
    parser.add_argument('--dataset-root', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--train-file', default='train.jsonl')
    parser.add_argument('--val-file', default='val.jsonl')
    parser.add_argument('--epochs', type=int, default=5)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--learning-rate', type=float, default=1e-3)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument('--image-width', type=int, default=160)
    parser.add_argument('--image-height', type=int, default=120)
    parser.add_argument('--max-tokens', type=int, default=32)
    parser.add_argument('--max-vocab-size', type=int, default=2048)
    parser.add_argument('--min-token-frequency', type=int, default=1)
    parser.add_argument('--num-workers', type=int, default=2)
    parser.add_argument('--device', default='auto', help='auto, cuda, or cpu')
    parser.add_argument('--seed', type=int, default=13)
    parser.add_argument('--limit-train-rows', type=int, default=None)
    parser.add_argument('--limit-val-rows', type=int, default=None)
    args = parser.parse_args()
    summary = train_local(args)
    print(_pretty_json(summary))


if __name__ == '__main__':
    main()
