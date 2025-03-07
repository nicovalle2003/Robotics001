#Superfuncion que plotea robot y dibuja trayectoriadef plot_robot_trajectory(robot, q_trajectory, drawing_mode='continuous',
def plot_robot_trajectory(robot, q_trajectory, limits=None, eeframe=True, jointaxes=False,
                         shadow=False, drawing_mode='continuous', traj_color='b',
                         drawing_color='r', drawing_threshold=0.01, dt=0.05, block=True):
    """
    Visualiza un robot siguiendo una trayectoria con trazado de la ruta del efector final.

    Parámetros:
    -----------
    robot : DHRobot
        El robot a visualizar
    q_trajectory : ndarray
        Matriz de configuraciones articulares (n_puntos x n_articulaciones)
    limits : list, opcional
        Límites de visualización [xmin, xmax, ymin, ymax, zmin, zmax]
    eeframe : bool, opcional
        Si se debe mostrar el marco del efector final
    jointaxes : bool, opcional
        Si se deben mostrar los ejes de las articulaciones
    shadow : bool, opcional
        Si se debe mostrar la sombra del robot
    drawing_mode : str, opcional
        Modo de dibujo: 'continuous' o 'segments'
    traj_color : str, opcional
        Color de la trayectoria completa
    drawing_color : str, opcional
        Color de los trazos de dibujo
    drawing_threshold : float, opcional
        Umbral de velocidad para el modo 'segments'
    dt : float, opcional
        Tiempo entre actualizaciones de la visualización
    block : bool, opcional
        Si se debe bloquear la ejecución hasta cerrar la ventana

    Retorna:
    --------
    env : PyPlot
        El entorno de visualización
    """
    from roboticstoolbox.backends.PyPlot import PyPlot
    import numpy as np

    # Inicializar el entorno PyPlot
    env = PyPlot()
    env.launch()

    # Calcular posiciones del efector final para toda la trayectoria
    all_positions = []
    for q in q_trajectory:
        T = robot.fkine(q)
        all_positions.append(T.t)

    all_positions = np.array(all_positions)

    # Calcular velocidades para el modo 'segments'
    if drawing_mode == 'segments':
        velocities = np.zeros(len(all_positions))
        for i in range(1, len(all_positions)):
            velocities[i] = np.linalg.norm(all_positions[i] - all_positions[i-1]) / dt

    # Configurar los límites de visualización
    if limits:
        env.ax.set_xlim([limits[0], limits[1]])
        env.ax.set_ylim([limits[2], limits[3]])
        env.ax.set_zlim([limits[4], limits[5]])
    else:
        # Configurar automáticamente
        min_pos = np.min(all_positions, axis=0) - 0.5
        max_pos = np.max(all_positions, axis=0) + 0.5
        env.ax.set_xlim([min_pos[0], max_pos[0]])
        env.ax.set_ylim([min_pos[1], max_pos[1]])
        env.ax.set_zlim([min_pos[2], max_pos[2]])

    # Añadir el robot al entorno con las opciones especificadas
    env.add(robot, eeframe=eeframe, jointaxes=jointaxes, shadow=shadow)

    # Lista para almacenar los puntos de la trayectoria
    traj_points = []

    # Variables para el seguimiento de los trazos de dibujo
    drawing_points = []
    is_drawing = False

    # Objeto de línea para la trayectoria
    line_obj = None

    # Recorrer la trayectoria y actualizar la visualización
    for i in range(len(q_trajectory)):
        # Actualizar la configuración del robot
        robot.q = q_trajectory[i]

        # Obtener la posición actual del efector final
        T_current = robot.fkine(robot.q)
        current_pos = T_current.t
        traj_points.append(current_pos)

        # Determinar si está dibujando según el modo
        if drawing_mode == 'continuous':
            # En modo continuo, siempre está dibujando
            is_drawing = True
            drawing_points.append(current_pos)

        elif drawing_mode == 'segments':
            # En modo segmentos, dibuja cuando la velocidad es baja
            current_drawing = velocities[i] < drawing_threshold

            # Si cambia el estado de dibujo
            if current_drawing != is_drawing:
                is_drawing = current_drawing
                if is_drawing:
                    drawing_points = [current_pos]
                else:
                    # Dibujar el segmento completado
                    if len(drawing_points) > 1:
                        points_array = np.array(drawing_points)
                        env.ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2],
                                   f'{drawing_color}-', linewidth=2)

            # Añadir el punto actual si está dibujando
            if is_drawing:
                drawing_points.append(current_pos)

        # Dibujar la trayectoria completa hasta el momento
        if len(traj_points) > 1:
            points_array = np.array(traj_points)

            # Si ya existe una línea, eliminarla
            if line_obj:
                line_obj[0].remove()

            # Crear una nueva línea con todos los puntos acumulados
            line_obj = env.ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2],
                                  f'{traj_color}-', linewidth=1, alpha=0.5)

        # Actualizar la visualización
        env.step(dt)

    # Dibujar el último segmento si está en modo segmentos
    if drawing_mode == 'segments' and is_drawing and len(drawing_points) > 1:
        points_array = np.array(drawing_points)
        env.ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2],
                   f'{drawing_color}-', linewidth=2)

    # Si está en modo continuo, dibujar toda la trayectoria con el color de dibujo
    if drawing_mode == 'continuous' and len(drawing_points) > 1:
        points_array = np.array(drawing_points)
        env.ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2],
                   f'{drawing_color}-', linewidth=2)

    print("Trayectoria completada")

    # Mantener la visualización si block=True
    if block:
        env.hold()

    return env


#***EJEMPLO DE USO***
#Antes
#p_lim=[-1, 1, -1, 1, -0.15, 1.5]
#robot.plot(q=qtraj, limits=p_lim, eeframe=True, jointaxes=False, shadow=True ,backend='pyplot', block=True, dt=0.15)
#Ahora
#p_lim=[-1, 1, -1, 1, -0.15, 1.5]
# #plot_robot_trajectory(
#     #robot=robot,
#     q_trajectory=qtraj,
#     limits=p_lim,
#     eeframe=True,
#     jointaxes=False,
#     shadow=True,
#     drawing_mode='continuous',  # o 'segments' si prefieres
#     traj_color='r',             # Color de la trayectoria completa
#     drawing_color='b',          # Color del trazo principal
#     dt=0.15,
#     block=True
# )