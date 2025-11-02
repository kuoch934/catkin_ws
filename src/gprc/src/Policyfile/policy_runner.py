# policy_runner.py
# 載入並執行 RL 策略（policy）」的小幫手，把它跟 policy.pt/onnx 放同資料夾
from pathlib import Path
import numpy as np
import torch
from typing import Optional

# load and run RL policy
class LoadedPolicy:
    def __init__(self, folder: Optional[str] = None,
                 ts_name: str = "policy.pt", onnx_name: str = "policy.onnx"):
        p = Path(folder) if folder else Path(__file__).parent
        self.ts_path = (p / ts_name).resolve()
        self.onnx_path = (p / onnx_name).resolve()

        # 1) 載入 TorchScript 模型
        self.model = torch.jit.load(self.ts_path.as_posix(), map_location="cpu").eval()

        # 2) 取得 obs/act 維度：優先讀 ONNX，否則用權重形狀推測（適用常見 MLP）
        self.obs_dim = None
        self.act_dim = None
        # 優先走 ONNX：讀取 ONNX 的 I/O shape
        try:
            import onnx, onnx.shape_inference as si
            if self.onnx_path.exists():
                m = si.infer_shapes(onnx.load(self.onnx_path.as_posix()))
                self.obs_dim = int(m.graph.input[0].type.tensor_type.shape.dim[1].dim_value)
                self.act_dim = int(m.graph.output[0].type.tensor_type.shape.dim[1].dim_value)
        except Exception:
            pass
        # 備援方案：若沒有 ONNX 或讀不到 shape，從 TorchScript 的 線性層權重形狀推估：(Isaac Lab 常見 policy 是 MLP）
        if self.obs_dim is None:
            W = [v.shape for v in self.model.state_dict().values()
                 if hasattr(v, "ndim") and v.ndim == 2]
            if W:
                self.obs_dim, self.act_dim = int(W[0][1]), int(W[-1][0])

    # 推論介面
    def __call__(self, obs_vec: np.ndarray) -> np.ndarray:
        """obs_vec: shape=(obs_dim,) 的 numpy 向量"""
        x = torch.from_numpy(obs_vec.astype("float32")).unsqueeze(0)
        with torch.no_grad():
            y = self.model(x).squeeze(0).cpu().numpy()
        return y

if __name__ == "__main__":
    pi = LoadedPolicy()                               # 會自動找同資料夾的 policy.pt/onnx
    print(f"obs_dim={pi.obs_dim}, act_dim={pi.act_dim}")
    dummy = np.zeros(pi.obs_dim, dtype=np.float32)    # 假觀測
    act = pi(dummy)
    print("forward OK, act.shape =", act.shape)
