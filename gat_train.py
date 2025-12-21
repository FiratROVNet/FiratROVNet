from FiratROVNet.gat import Train, optimize_hyperparameters

# Senaryo verileriyle eğitim
Train(epochs=5000, use_senaryo=True)

# Hiperparametre optimizasyonu (Optuna ile)
best_params = optimize_hyperparameters(n_trials=20, epochs_per_trial=1000)

# Özel hiperparametrelerle eğitim
Train(epochs=5000, hidden_channels=24, num_heads=6, dropout=0.2, use_senaryo=True)
