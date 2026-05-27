"""
soup_plant.py
=============
Generalized Soup Plant Model for Adaptive Control (STR) Simulation

【物理モデルのポイント】
1. 液体（水＋味噌）と固形物（ポテトまたは豆腐）の二槽熱モデル。
2. 濃度 C [%] は、論文の厳密な定義「追加した塩の総量 / スープ固有の水質量 * 100」で計算。
3. 未知のスープとしてコントローラーに挑ませるため、プラント内部に「真の値」を持たせます。
"""

COMMON_CONSTANTS = {"T_BASE": 25.0, "A_COEFF": 0.02}
SOUP_CONFIG = {
    "water": {
        "water_mass": 600.0, "miso_mass": 0.0, "potato_mass": 0.0, "tofu_mass": 0.0,
        "Qin": 0.2932, "k_env": 0.0096, "k_loss": 0.0, "k_absorb": 0.0,
        "alpha": 11.9320, "beta": 0.3410  # 👈 ロボットの初期予習知識と同じ
    },
    "potato": {
        "water_mass": 500.0, "miso_mass": 0.0, "potato_mass": 100.0, "tofu_mass": 0.0,
        "Qin": 0.2932, "k_env": 0.0096, "k_loss": 0.0017, "k_absorb": 0.0098,
        # 💡【修正】ポテトの特性を水から大きく離し、PIDを狂わせる設定
        "alpha": 8.5000,
        "beta":  1.5000
    },
    "miso_tofu": {
        "water_mass": 490.0, "miso_mass": 10.0, "potato_mass": 0.0, "tofu_mass": 100.0,
        "Qin": 0.2932, "k_env": 0.0096, "k_loss": 0.0033, "k_absorb": 0.0171,
        "alpha": 10.8500, "beta": 5.1200
    }
}

class SoupPlant:
    """
    リアルな実験環境（鍋）を模したスーププラントクラス。
    """
    def __init__(self, 
                 soup_type: str = "potato", 
                 T_w_init: float = 20.0, 
                 T_s_init: float = 20.0, 
                 T_room: float = 20.0):
        
        if soup_type not in SOUP_CONFIG:
            raise ValueError(f"Unknown soup_type: {soup_type}. Choose from 'water', 'potato', 'miso_tofu'.")
            
        cfg = SOUP_CONFIG[soup_type]
        self.soup_type = soup_type
        
        # --- 🍲 スープを構成する各物質の質量 [g] ---
        self.m_water = cfg["water_mass"]
        self.m_miso = cfg["miso_mass"]
        self.m_potato = cfg["potato_mass"]
        self.m_tofu = cfg["tofu_mass"]
        
        # 食品物理学に基づく比熱定数 [J/(g・K)]
        self.c_water = 4.18
        self.c_miso = 2.50
        self.c_potato = 3.60
        self.c_tofu = 3.90
        
        # --- 🌡️ 状態変数（シミュレーションで毎秒変化する値） ---
        self.T_w = T_w_init       # 液体（スープ）の温度 [°C]
        self.T_s = T_s_init       # 固形物（具材）の温度 [°C]
        
        # 💡 味噌の初期塩分：味噌ペーストの約10%が純粋な塩分(NaCl)として液相に溶けていると定義
        MISO_SALT_RATIO = 0.10
        self.salt_mass = self.m_miso * MISO_SALT_RATIO 
        
        # --- ⚙️ 物理環境定数のロード ---
        self.T_room = T_room
        self.T_base = COMMON_CONSTANTS["T_BASE"]
        self.a = COMMON_CONSTANTS["A_COEFF"]
        
        # 実験から特定された熱パラメータ
        self.k_env = cfg["k_env"]
        self.k_loss = cfg["k_loss"]
        
        # --- 🧠 物理拘束に基づいた k_absorb の動的計算 ---
        # 液体全体の熱容量（水 ＋ 溶けた味噌）
        C_liquid = (self.m_water * self.c_water) + (self.m_miso * self.c_miso)
        
        # 固形物全体の熱容量（ポテト または 豆腐）
        if self.m_potato > 0:
            C_solid = self.m_potato * self.c_potato
            self.has_solid = True
        elif self.m_tofu > 0:
            C_solid = self.m_tofu * self.c_tofu
            self.has_solid = True
        else:
            C_solid = 1.0  # ゼロ除算防止
            self.has_solid = False
            
        # エネルギー保存則を満たすように吸熱係数を決定
        if self.has_solid:
            self.k_absorb = (C_liquid / C_solid) * self.k_loss
        else:
            self.k_absorb = 0.0
            
        # 実験からフィッティングされた「真の」伝導率係数（コントローラーには隠蔽する）
        self.alpha = cfg["alpha"]
        self.beta = cfg["beta"]

    def step(self, Q_in: float, salt_added: float = 0.0, dt: float = 1.0):
        """
        1ステップ（dt秒）時間を進める（前進オイラー法）
        
        Parameters:
        -----------
        Q_in       : ヒーターからの熱入力
        salt_added : この1秒間で追加された塩の質量 [g]
        dt         : 時間刻み [s]
        """
        # --- 1. 熱力学モデルの更新 ---
        if self.has_solid:
            # 具材がある場合（相互の熱移動を計算）
            dT_w = (Q_in 
                    - self.k_env  * (self.T_w - self.T_room) 
                    - self.k_loss * (self.T_w - self.T_s))
            dT_s = self.k_absorb * (self.T_w - self.T_s)
            
            self.T_w += dT_w * dt
            self.T_s += dT_s * dt
        else:
            # 純水の場合（具材への逃げがない）
            dT_w = Q_in - self.k_env * (self.T_w - self.T_room)
            self.T_w += dT_w * dt
            
        # --- 2. 塩分質量の更新 ---
        self.salt_mass += salt_added

    def reset(self, T_w_init: float = 20.0, T_s_init: float = 20.0):
        """プラントの状態を初期化する"""
        self.T_w = T_w_init
        self.T_s = T_s_init
        MISO_SALT_RATIO = 0.10
        self.salt_mass = self.m_miso * MISO_SALT_RATIO

    # ==========================================
    # センサー出力（プロパティ）
    # ==========================================
    @property
    def concentration(self) -> float:
        """【真値】実際の塩分濃度 C [%] （定義：総塩分質量 ➗ 固有の水質量 × 100）"""
        if self.m_water == 0:
            return 0.0
        return (self.salt_mass / self.m_water) * 100.0

    @property
    def conductivity(self) -> float:
        """【センサー実測値】電気伝導率 σ [mS/cm]（温度変化の影響を含む）"""
        C = self.concentration
        return (self.alpha * C + self.beta) * (1.0 + self.a * (self.T_w - self.T_base))

    def state(self) -> dict:
        """コントローラーやログ保存用に、現在の実測値を辞書で返す"""
        return {
            "soup_type": self.soup_type,
            "T_w":       self.T_w,                               # 液温センサー値
            "T_s":       self.T_s if self.has_solid else None,   # 具材温度（不可視でも可）
            "sigma":     self.conductivity,                      # 電気伝導率センサー値
            "salt_mass": self.salt_mass,                         # 現在の総塩分
            "C_true":    self.concentration                      # 【検証用】本当の濃度
        }