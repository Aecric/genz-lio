Vendored subset of [GenZ-ICP](https://github.com/cocel-postech/genz-icp) (MIT) at commit 674ccce,
copied from Super-LIO `thirdparty/genz_icp`: `core/VoxelHashMap.{hpp,cpp}` and `core/Registration.{hpp,cpp}`.

- `sophus/` — Sophus (MIT), the pre-fmt header-only revision also used by Super-LIO / lightning-lm.
- tsl::robin_map is not vendored here: the GenZ-LIO core already ships it in `cpp/genz_lio/3rdparty/tsl`.
