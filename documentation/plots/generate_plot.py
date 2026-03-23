import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# --- Configuración Dinámica de Rutas ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))

# Cambiamos el nombre de la salida para diferenciarla
IMAGE_NAME = "plan_min_energy_comparison.png"
OUTPUT_PATH = os.path.join(REPO_PATH, "documentation", "plots", "output", IMAGE_NAME)

INPUT_PATH1 = os.path.join(REPO_PATH, "simulation_data", "short_example", "execution_data", "plan", "min_energy", "solution_path.csv")
INPUT_PATH2 = os.path.join(REPO_PATH, "simulation_data", "short_example", "execution_data", "plan", "min_energy", "pure_pursuit", "lookahead=1", "executed_path.csv")
INPUT_PATH3 = os.path.join(REPO_PATH, "simulation_data", "short_example", "execution_data", "plan", "min_energy", "pose_pid", "executed_path.csv") 

def plot_xy_position():
    """
    Lee 3 archivos CSV y grafica la posición Y frente a la posición X (Vista 2D).
    """
    print(f"Generando gráfica de posición Y(x) [Vista de pájaro]...")

    # 1. Leer los CSV
    rutas = [INPUT_PATH1, INPUT_PATH2, INPUT_PATH3]
    dfs = []
    
    for i, ruta in enumerate(rutas, start=1):
        if not os.path.exists(ruta):
            print(f"❌ Error: No se encontró el archivo {i}.\nRuta buscada: {ruta}")
            return
        dfs.append(pd.read_csv(ruta))

    df1, df2, df3 = dfs

    # Verificar que las columnas 'x' e 'y' existen
    columnas_necesarias = ['x', 'y']
    for i, df in enumerate([df1, df2, df3], start=1):
        for col in columnas_necesarias:
            if col not in df.columns:
                print(f"❌ Error: La columna '{col}' no existe en el archivo {i}")
                return

    # 2. Configurar la figura (usamos una figura más cuadrada para mejor visualización 2D)
    plt.figure(figsize=(10, 8))

    # 3. Graficar Y(x)
    
    # Ruta Ideal (Waypoints státicos). Usamos marcadores para que se vea claro.
    plt.plot(df1['x'], df1['y'], label='Planned path', color='#2ca02c', linewidth=2, linestyle='--', marker='o', markersize=5)
    
    # Ejecuciones reales (Línea continua sin marcadores para no saturar)
    plt.plot(df2['x'], df2['y'], label='Pure Pursuit', color='#ff7f0e', linewidth=2.5, alpha=0.9)
    plt.plot(df3['x'], df3['y'], label='Pose PID', color='#1f77b4', linewidth=2.5, alpha=0.9)

    # 4. Personalizar la gráfica
    plt.xlabel('Position X (m)', fontsize=12)
    plt.ylabel('Position Y (m)', fontsize=12)
    
    # --- FUNDAMENTAL PARA ROBÓTICA ---
    # Hace que la escala de los ejes X e Y sea idéntica, 
    # para que un círculo se vea como un círculo y no como una elipse.
    plt.axis('equal')
    
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()

    # 5. Guardar y mostrar
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    plt.savefig(OUTPUT_PATH, dpi=300)
    print(f"✅ Gráfica de posición XY guardada exitosamente en: {OUTPUT_PATH}")
    
    plt.show()

if __name__ == "__main__":
    # Ya no necesitamos CAMPO_A_GRAFICAR, usamos 'x' e 'y' directamente.
    plot_xy_position()