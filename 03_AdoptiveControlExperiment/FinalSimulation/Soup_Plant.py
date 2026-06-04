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
        "alpha": 8.1013, "beta": 0.155  # 👈 ロボットの初期予習知識と同じ
    },
    "miso": {
        "water_mass": 580.0, "miso_mass": 20.0, "potato_mass": 0.0, "tofu_mass": 0.0,
        "Qin": 0.2932, "k_env": 0.0096, "k_loss": 0.0017, "k_absorb": 0.0098,
        # 💡【修正】ポテトの特性を水から大きく離し、PIDを狂わせる設定
        "alpha": 9.1819,
        "beta":  1.1606
    },
    "miso_tofu": {
        "water_mass": 480.0, "miso_mass": 20.0, "potato_mass": 0.0, "tofu_mass": 100.0,
        "Qin": 0.2932, "k_env": 0.0096, "k_loss": 0.0033, "k_absorb": 0.0171,
        "alpha": 8.5359, "beta": 1.1373
    }
}

class SoupPlant:
    """リアルな実験環境（鍋）を模したスーププラントクラス。"""
    def __init__(self, 
                 soup_type: str = "miso", 
                 T_w_init: float = 20.0, 
                 T_s_init: float = 20.0, 
                 T_room: float = 24.6):
        
        if soup_type not in SOUP_CONFIG:
            raise ValueError(f"Unknown soup_type: {soup_type}. Choose from 'water', 'miso', 'miso_tofu'.")
            
        cfg = SOUP_CONFIG[soup_type]
        self.soup_type = soup_type
        
        # --- 🍲 スープを構成する各物質の質量 [g] ---
        self.m_water = cfg["water_mass"]
        self.m_miso  = cfg["miso_mass"]
        self.m_tofu  = cfg["tofu_mass"]
        
        # ベース液相の総質量（濃度計算の分母の基準）
        self.m_base_liquid = self.m_water + self.m_miso
        
        # 食品物理学に基づく比熱定数 [J/(g・K)]
        self.c_water = 4.18
        self.c_miso  = 2.50
        self.c_tofu  = 3.90
        
        # --- 🌡️ 状態変数 ---
        self.T_w = T_w_init
        self.T_s = T_s_init
        
        # 味噌の初期塩分：味噌ペーストの 8.7% が塩分(NaCl)と定義
        MISO_SALT_RATIO = 0.087
        self.initial_salt = self.m_miso * MISO_SALT_RATIO 
        self.salt_mass    = self.initial_salt
        self.added_salt   = 0.0
        
        # --- ⚙️ 物理環境定数 ---
        self.T_room = T_room
        self.T_base = COMMON_CONSTANTS["T_BASE"]
        self.a      = COMMON_CONSTANTS["A_COEFF"]
        
        self.k_env  = cfg["k_env"]
        self.k_loss = cfg["k_loss"]
        
        # --- 🧠 物理拘束に基づいた k_absorb の動的計算 ---
        C_liquid = (self.m_water * self.c_water) + (self.m_miso * self.c_miso)
        if self.m_tofu > 0:
            C_solid = self.m_tofu * self.c_tofu
            self.has_solid = True
            self.k_absorb = (C_liquid / C_solid) * self.k_loss
        else:
            self.has_solid = False
            self.k_absorb = 0.0
            
        # 真のセンサー特性
        self.alpha = cfg["alpha"]
        self.beta  = cfg["beta"]

    def step(self, Q_in: float, salt_added_this_step: float = 0.0, dt: float = 1.0):
        """1ステップ（dt秒）時間を進める"""
        # 1. 熱力学モデルの更新
        if self.has_solid:
            dT_w = (Q_in 
                    - self.k_env  * (self.T_w - self.T_room) 
                    - self.k_loss * (self.T_w - self.T_s))
            dT_s = self.k_absorb * (self.T_w - self.T_s)
            self.T_w += dT_w * dt
            self.T_s += dT_s * dt
        else:
            dT_w = Q_in - self.k_env * (self.T_w - self.T_room)
            self.T_w += dT_w * dt
            
        # 2. 塩分質量の更新
        self.added_salt += salt_added_this_step
        self.salt_mass  = self.initial_salt + self.added_salt

    def reset(self, T_w_init: float = 20.0, T_s_init: float = 20.0):
        """プラントの状態を初期化する"""
        self.T_w = T_w_init
        self.T_s = T_s_init
        MISO_SALT_RATIO = 0.087
        self.initial_salt = self.m_miso * MISO_SALT_RATIO
        self.salt_mass    = self.initial_salt
        self.added_salt   = 0.0

    @property
    def concentration(self) -> float:
        """【真値】実際の塩分濃度 C [%]（実機解析コードと完全一致）"""
        if self.m_base_liquid == 0:
            return 0.0
        return (self.salt_mass / (self.m_base_liquid + self.added_salt)) * 100.0

    @property
    def conductivity(self) -> float:
        """【センサー実測値】電気伝導率 σ [mS/cm]（温度変化の影響を含む）"""
        C = self.concentration
        return (self.alpha * C + self.beta) * (1.0 + self.a * (self.T_w - self.T_base))

    def state(self) -> dict:
        """現在の実測値を辞書で返す"""
        return {
            "soup_type": self.soup_type,
            "T_w":       self.T_w,
            "sigma":     self.conductivity,
            "salt_mass": self.salt_mass,
            "C_true":    self.concentration
        }