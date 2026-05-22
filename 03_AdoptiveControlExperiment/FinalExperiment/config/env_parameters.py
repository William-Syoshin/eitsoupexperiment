# ==========================================
# 03_AdaptiveControlExperiment - Environment Parameters
# ==========================================
# This file stores all the fitted parameters identified from the experiments.
# Standard reference temperature: 25.0 C

SOUP_CONFIG = {
    # ------------------------------------------
    # 1. Pure Water (Baseline)
    # ------------------------------------------
    "water": {
        "water_mass": 600.0,
        "miso_mass": 0.0,
        "solid_mass": 0.0,
        
        # Thermal dynamics
        "Qin": 0.2932,
        "k_env": 0.0096,
        "k_loss": 0.0,      # No ingredients
        "k_absorb": 0.0,    # No ingredients
        
        # Conductivity parameters
        "alpha": 11.9320,   # Example fitted value
        "beta": 0.3410,     # Example fitted value
    },
    
    # ------------------------------------------
    # 2. Potato Soup
    # ------------------------------------------
    "potato": {
        "water_mass": 500.0,
        "miso_mass": 0.0,
        "solid_mass": 100.0,
        
        # Thermal dynamics
        "Qin": 0.2932,
        "k_env": 0.0096,
        "k_loss": 0.0017,
        "k_absorb": 0.0098,
        
        # Conductivity parameters
        "alpha": 9.6679,   # Put your actual fitted alpha here
        "beta": 0.3069,     # Put your actual fitted beta here
    },
    
    # ------------------------------------------
    # 3. Miso Soup with Tofu
    # ------------------------------------------
    "miso_tofu": {
        "water_mass": 465.0,
        "miso_mass": 35.0,
        "solid_mass": 100.0,
        
        # Thermal dynamics
        "Qin": 0.2932,
        "k_env": 0.0096,
        "k_loss": 0.0033,
        "k_absorb": 0.0171,
        
        # Conductivity parameters
        "alpha": 9.0121,   # Put your actual fitted alpha here
        "beta": 3.8048,     # Put your actual fitted beta here (Should be high!)
    }
}

# Common constants across all experiments
COMMON_CONSTANTS = {
    "T_BASE": 25.0,      # Reference temperature for conductivity (C)
    "A_COEFF": 0.02,     # Temperature coefficient for conductivity (per C)
    "TOTAL_WEIGHT": 600.0 # Total target weight of the soup (g)
}