# Sources and third-party notices

The project runs pretrained locomotion inference. It does not train or claim authorship of the upstream policy.

- Go2 model: [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), commit `bd9709b540d58e1dcf417e4ffeffc7d54318280d`; original `unitree_go2` files copied into `models/go2`. License: `models/go2/LICENSE` (BSD 3-Clause terms). Exact hashes: `local_model_manifest.json`.
- Locomotion policy and configuration: [Teddy-Liao/walk-these-ways-go2](https://github.com/Teddy-Liao/walk-these-ways-go2), commit `d00320778c61567379b4a304047955c0c0dcae43`. License: `assets/WTW_LICENSE` (MIT). Checkpoints and original parameter file are documented in `download_manifest.json`. Normal execution reads the extracted `policy_config.json`, not the pickle.
- Observation / deployment conventions: [art-e-fact/unitree_mujoco](https://github.com/art-e-fact/unitree_mujoco), commit `fcc708030457e2f95ae7a23db7cc5e8585e7bc8e`. The applicable BSD notice is retained in `assets/controller_LICENSE`.
- Related research, not an implementation reproduced here: Kim et al., “Transforming a Quadruped into a Guide Robot for the Visually Impaired: Formalizing Wayfinding, Interaction Modeling, and Safety Mechanism,” CoRL 2023. [Paper](https://arxiv.org/abs/2306.14055). Its measured interaction model and physical harness are not present in this project.
- Simulator: [MuJoCo](https://github.com/google-deepmind/mujoco), installed as a Python dependency. Dependency licenses remain with their packages.

Custom glue, experiments, documentation and video assembly were developed with Codex assistance. Original notices must be preserved when redistributing source or assets. No upstream organization endorses this project.
