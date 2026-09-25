# warsaw_hackathon

Unitree G1 side of the Alien Bazaar hackathon project. Architecture and rules: [AGENTS.md](AGENTS.md).

## Clone

The repo has two submodules:

- `third_party/vggt`: VGGT model code ([facebookresearch/vggt](https://github.com/facebookresearch/vggt))
- `VGGT-1B`: VGGT-1B weights ([facebook/VGGT-1B](https://huggingface.co/facebook/VGGT-1B)), Git LFS, ~10 GB, CC-BY-NC-4.0

### Machine without a GPU (does not run VGGT)

Skip the weights, then stop later pulls from fetching them:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --recurse-submodules <repo-url>
git -C warsaw_hackathon/VGGT-1B config lfs.fetchexclude '*'
```

Already cloned:

```bash
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive
git -C VGGT-1B config lfs.fetchexclude '*'
```

`VGGT-1B/model.*` then stay as ~135-byte LFS pointer files.

### Machine that runs VGGT

Fetch only the weights file the model code loads (`VGGT.from_pretrained` uses `model.safetensors`):

```bash
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive
git -C VGGT-1B lfs pull --include model.safetensors
```
