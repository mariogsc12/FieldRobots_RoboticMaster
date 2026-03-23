import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# --- Configuración Dinámica de Rutas ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))

# --- Rutas de Salida ---
# Cambiamos el nombre de la salida para diferenciarla
IMAGE_BASE_NAME = "direct_comparison"
PATH_IMAGE_NAME = f"{IMAGE_BASE_NAME}_path.png"
TWIST_V_IMAGE_NAME = f"{IMAGE_BASE_NAME}_linear_velocity.png"
TWIST_W_IMAGE_NAME = f"{IMAGE_BASE_NAME}_angular_velocity.png"

OUTPUT_PATH_XY = os.path.join(REPO_PATH, "documentation", "plots", "twist", "output", PATH_IMAGE_NAME)
OUTPUT_PATH_TWIST_V = os.path.join(REPO_PATH, "documentation", "plots", "twist", "output", TWIST_V_IMAGE_NAME)
OUTPUT_PATH_TWIST_W = os.path.join(REPO_PATH, "documentation", "plots", "twist", "output", TWIST_W_IMAGE_NAME)

# --- Rutas de Entrada (Posición) ---
BASE_INPUT_DIR = os.path.join(REPO_PATH, "simulation_data", "short_example", "execution_data", "direct")

INPUT_PATH1 = os.path.join(BASE_INPUT_DIR, "solution_path.csv")
INPUT_PATH2 = os.path.join(BASE_INPUT_DIR, "pure_pursuit", "lookahead=1", "executed_path.csv")
INPUT_PATH3 = os.path.join(BASE_INPUT_DIR, "pose_pid", "executed_path.csv") 

# --- Rutas de Entrada (Twist / Velocidades) ---
TWIST_PATH2 = os.path.join(BASE_INPUT_DIR, "pure_pursuit", "lookahead=1", "executed_twist.csv")
TWIST_PATH3 = os.path.join(BASE_INPUT_DIR, "pose_pid", "executed_twist.csv")

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
    plt.figure(figsize=(14, 6))

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
    os.makedirs(os.path.dirname(OUTPUT_PATH_XY), exist_ok=True)
    plt.savefig(OUTPUT_PATH_XY, dpi=300)
    print(f"✅ Gráfica de posición XY guardada exitosamente en: {OUTPUT_PATH_XY}")
    
    plt.show()

def plot_twist_comparison():
    """
    Lee los archivos de twist y grafica la velocidad lineal (v) 
    y angular (w) en dos imágenes separadas.
    """
    print("Generando gráficas de Twist (Velocidades v y w separadas)...")

    # 1. Leer los CSV de Twist
    rutas = [TWIST_PATH2, TWIST_PATH3]
    dfs = []
    
    for i, ruta in enumerate(rutas, start=2):
        if not os.path.exists(ruta):
            print(f"❌ Error: No se encontró el archivo de twist {i}.\nRuta buscada: {ruta}")
            return
        dfs.append(pd.read_csv(ruta))

    df2, df3 = dfs

    # Verificar que las columnas 'v' y 'w' existen
    for i, df in enumerate([df2, df3], start=2):
        if 'v' not in df.columns or 'w' not in df.columns:
            print(f"❌ Error: Las columnas 'v' o 'w' no existen en el archivo de twist {i}")
            return

    # --- NORMALIZACIÓN DE PASOS (0 a 100%) ---
    eje_x_2 = np.linspace(0, 100, len(df2))
    eje_x_3 = np.linspace(0, 100, len(df3))

    os.makedirs(os.path.dirname(OUTPUT_PATH_TWIST_V), exist_ok=True)

    # ---------------------------------------------------------
    # IMAGEN 1: VELOCIDAD LINEAL (v)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 5))
    plt.plot(eje_x_2, df2['v'], label='Pure Pursuit', color='#ff7f0e', linewidth=2, alpha=0.9)
    plt.plot(eje_x_3, df3['v'], label='Pose PID', color='#1f77b4', linewidth=2, alpha=0.9)
    
    plt.xlabel('Simulation progress (%)', fontsize=12)
    plt.ylabel('Linear velocity ($v$)', fontsize=12)
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()

    plt.savefig(OUTPUT_PATH_TWIST_V, dpi=300)
    print(f"✅ Gráfica de Velocidad Lineal guardada en: {OUTPUT_PATH_TWIST_V}")
    plt.show() # Mostrará la primera ventana

    # ---------------------------------------------------------
    # IMAGEN 2: VELOCIDAD ANGULAR (w)
    # ---------------------------------------------------------
    plt.figure(figsize=(10, 5))
    plt.plot(eje_x_2, df2['w'], label='Pure Pursuit', color='#ff7f0e', linewidth=2, alpha=0.9)
    plt.plot(eje_x_3, df3['w'], label='Pose PID', color='#1f77b4', linewidth=2, alpha=0.9)
    
    plt.xlabel('Simulation progress (%)', fontsize=12)
    plt.ylabel('Angular velocity ($w$)', fontsize=12)
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()

    plt.savefig(OUTPUT_PATH_TWIST_W, dpi=300)
    print(f"✅ Gráfica de Velocidad Angular guardada en: {OUTPUT_PATH_TWIST_W}")
    plt.show() # Mostrará la segunda ventana tras cerrar la primera

if __name__ == "__main__":
    # Descomenta las funciones que quieras ejecutar
    
    plot_xy_position()
    # plot_twist_comparison()