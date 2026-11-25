"""
SEA-RAFT (Wang, Lipson and Deng, ECCV 2024), the network modules only, for inference.

Vendored from princeton-vl/SEA-RAFT @ 9137517 (BSD-3-Clause, see LICENSE in this folder):
core/raft.py, core/extractor.py, core/layer.py and core/update.py. The module names are
unchanged, so the released checkpoints load strictly. The inference loop is in
rapid_nav_perception/searaft.py.
"""

from rapid_nav_perception.nets.searaft.raft import searaft_args, SeaRaftNet

__all__ = ['SeaRaftNet', 'searaft_args']
