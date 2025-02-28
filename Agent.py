#... rest of the code
def think_and_send(self):
            w = self.world
            r = self.world.robot  
            my_head_pos_2d = r.loc_head_position[:2]
            my_ori = r.imu_torso_orientation
            ball_2d = w.ball_abs_pos[:2]
            ball_vec = ball_2d - my_head_pos_2d
            ball_dir = M.vector_angle(ball_vec)
            ball_dist = np.linalg.norm(ball_vec)
            ball_sq_dist = ball_dist * ball_dist # for faster comparisons
            ball_speed = np.linalg.norm(w.get_ball_abs_vel(6)[:2])
            behavior = self.behavior
            goal_dir = M.target_abs_angle(ball_2d,(15.05,0))
            path_draw_options = self.path_manager.draw_options
            PM = w.play_mode
            PM_GROUP = w.play_mode_group

            #--------------------------------------- 1. Preprocessing

            # Obtén la posición predicha de la bola
            slow_ball_pos = w.get_predicted_ball_pos(0.5)  # Predicción cuando la velocidad <= 0.5 m/s

            # Definir una posición por defecto para jugadores sin información (se usará para evitar errores)
            default_pos = np.array([1000, 1000]) 
            
            # --- Vectorización para compañeros ---
            # Extraer posiciones: si state_abs_pos es None, se usa default_pos
            teammate_positions = np.array([
                np.array(p.state_abs_pos[:2]) if p.state_abs_pos is not None else default_pos
                for p in w.teammates
            ])
            # Extraer otros atributos con comprobación para evitar None
            teammate_last_updates = np.array([
                p.state_last_update if p.state_last_update is not None else 0
                for p in w.teammates
            ])
            teammate_is_self = np.array([p.is_self for p in w.teammates])
            teammate_fallen = np.array([p.state_fallen for p in w.teammates])
            # Incluimos en la máscara también que la posición sea válida
            valid_mask = (
                (teammate_last_updates != 0) &
                (((w.time_local_ms - teammate_last_updates) <= 360) | teammate_is_self) &
                (~teammate_fallen) &
                (np.array([p.state_abs_pos is not None for p in w.teammates]))
            )
            # Calcula las diferencias y la norma al cuadrado
            diff_teammates = teammate_positions - slow_ball_pos
            sq_distances_teammates = np.sum(diff_teammates**2, axis=1)
            # Para los jugadores con datos inválidos, asigna un valor grande (1000)
            sq_distances_teammates[~valid_mask] = 1000
            teammates_ball_sq_dist = sq_distances_teammates.tolist()

            # --- Vectorización para oponentes ---
            opponent_positions = np.array([
                np.array(p.state_abs_pos[:2]) if p.state_abs_pos is not None else default_pos
                for p in w.opponents
            ])
            opponent_last_updates = np.array([
                p.state_last_update if p.state_last_update is not None else 0
                for p in w.opponents
            ])
            opponent_fallen = np.array([p.state_fallen for p in w.opponents])
            valid_mask_opponents = (
                (opponent_last_updates != 0) &
                ((w.time_local_ms - opponent_last_updates) <= 360) &
                (~opponent_fallen) &
                (np.array([p.state_abs_pos is not None for p in w.opponents]))
            )
            diff_opponents = opponent_positions - slow_ball_pos
            sq_distances_opponents = np.sum(diff_opponents**2, axis=1)
            sq_distances_opponents[~valid_mask_opponents] = 1000
            opponents_ball_sq_dist = sq_distances_opponents.tolist()

            # Calcular las distancias mínimas
            min_teammate_ball_sq_dist = np.min(teammates_ball_sq_dist)
            self.min_teammate_ball_dist = math.sqrt(min_teammate_ball_sq_dist)
            self.min_opponent_ball_dist = math.sqrt(np.min(opponents_ball_sq_dist))


            active_player_unum = teammates_ball_sq_dist.index(min_teammate_ball_sq_dist) + 1
#Rest of the code ...
