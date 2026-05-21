"""
soup_plant.py
=============
スープ プラントモデル（インポート用モジュール）

使い方:
    from soup_plant import SoupPlant

    plant = SoupPlant(water_mass=500, potato_mass=100)
    plant.step(Q_in=0.2, salt_added=0.0, dt=1.0)
    print(plant.T_w, plant.T_p, plant.concentration, plant.conductivity)
"""

class SoupPlant:
    """
    水＋芋スープのプラントモデル

    【熱モデル（二槽：水＋芋）】
      芋あり:
        dT_w/dt = Q_in - k_env·(T_w - T_room) - k_loss·(T_w - T_p)
        dT_p/dt = k_absorb·(T_w - T_p)
      芋なし:
        dT_w/dt = Q_in - k_env·(T_w - T_room)

      拘束条件（エネルギー保存）:
        k_absorb = (m_w·c_w) / (m_p·c_p) × k_loss

    【伝導率モデル】
      σ = (α·C + β)·(1 + a·(T_w - T_base))
      C = salt_mass / liquid_mass × 100  [%]
          ※ liquid_mass = water_mass + salt_mass（塩は液相のみ）
          ※ 芋は固相とみなし、濃度の分母には含めない
    """

    def __init__(self,
                 # --- スープ構成 ---
                 water_mass: float = 500.0,
                 potato_mass: float = 100.0,
                 c_water: float = 4.18,
                 c_potato: float = 3.50,
                 # --- 初期状態 ---
                 T_w_init: float = 20.0,
                 T_p_init: float = 20.0,
                 T_room: float = 20.0,
                 salt_init: float = 0.0,
                 # --- 熱モデル定数 ---
                 k_env: float = 0.0010,
                 k_loss: float = 0.0017,
                 # --- σモデル定数 ---

                 #water only 
                 #alpha: float = 8.836,
                 #beta: float = 0.499, 

                 #wate + potato
                 alpha: float = 11.925,
                 beta: float = 0.3390,

                 a: float = 0.02,
                 T_base: float = 25.0):

        # --- 構成 ---
        self.m_w = water_mass
        self.m_p = potato_mass
        self.c_w = c_water
        self.c_p = c_potato
        self.has_potato = potato_mass > 0

        # --- 状態 ---
        self.T_w = T_w_init
        self.T_p = T_p_init
        self.salt_mass = salt_init
        self.liquid_mass = water_mass + salt_init

        # --- 環境 ---
        self.T_room = T_room

        # --- 熱モデル定数 ---
        self.k_env = k_env
        self.k_loss = k_loss
        if self.has_potato:
            self.k_absorb = (self.m_w * self.c_w) / (self.m_p * self.c_p) * k_loss
        else:
            self.k_absorb = 0.0

        # --- σモデル定数 ---
        self.alpha = alpha
        self.beta = beta
        self.a = a
        self.T_base = T_base

    def step(self, Q_in: float, salt_added: float = 0.0, dt: float = 1.0):
        """
        1ステップ進める（前進オイラー法）

        Parameters
        ----------
        Q_in       : 水への熱入力 [°C/s]（水熱容量で正規化済み）
        salt_added : 塩添加量 [g]
        dt         : 時間刻み [s]
        """
        if self.has_potato:
            dT_w = (Q_in
                    - self.k_env  * (self.T_w - self.T_room)
                    - self.k_loss * (self.T_w - self.T_p))
            dT_p = self.k_absorb * (self.T_w - self.T_p)
            self.T_w += dT_w * dt
            self.T_p += dT_p * dt
        else:
            dT_w = Q_in - self.k_env * (self.T_w - self.T_room)
            self.T_w += dT_w * dt

        self.salt_mass   += salt_added
        self.liquid_mass += salt_added

    def reset(self,
              T_w_init: float = 20.0,
              T_p_init: float = 20.0,
              salt_init: float = 0.0):
        """状態を初期値にリセットする"""
        self.T_w = T_w_init
        self.T_p = T_p_init
        self.salt_mass = salt_init
        self.liquid_mass = self.m_w + salt_init

    @property
    def concentration(self) -> float:
        """塩分濃度 C [%]（液相中の質量比）"""
        return self.salt_mass / self.liquid_mass * 100.0

    @property
    def conductivity(self) -> float:
        """電気伝導率 σ [mS/cm]"""
        C = self.concentration
        return (self.alpha * C + self.beta) * (1.0 + self.a * (self.T_w - self.T_base))

    @property
    def estimated_concentration(self) -> float:
        """温度補償済み推定濃度 Ĉ [%]（σとT_wから逆算）"""
        sigma = self.conductivity
        temp_factor = 1.0 + self.a * (self.T_w - self.T_base)
        return (sigma / temp_factor - self.beta) / self.alpha

    def state(self) -> dict:
        """現在の全状態を辞書で返す"""
        return {
            "T_w":   self.T_w,
            "T_p":   self.T_p if self.has_potato else None,
            "C":     self.concentration,
            "C_hat": self.estimated_concentration,
            "sigma": self.conductivity,
            "salt":  self.salt_mass,
        }
