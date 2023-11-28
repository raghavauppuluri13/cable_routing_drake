# cable_routing

### Dependencies

- [poetry](https://python-poetry.org/docs)
- [drake](https://drake.mit.edu/)
- [git lfs](https://git-lfs.github.com/) - for meshes

### Quick Start
1. Clone repo with submodules and get LFS files

```
# clone
git clone https://github.com/raghavauppuluri/cable_routing_drake.git --recurse-submodules
git lfs install
git lfs fetch
git lfs pull
```

2. Install dependencies
```
poetry install
```

2. Activate shell and run
```
poetry shell
python3 cable_routing/env.py
```
