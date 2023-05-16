import numpy as np
from skimage.feature import peak_local_max

# -------------------- Maximum analysis functions --------------------

def find_video_local_maximums(video, known_maximums):
    # Finds all frames local maximums and returns maximums_points data base
    # INPUTS:
    # video:        images (or frames) sequence (numpy array) to be proceesed. Array shape
    #               must be axbxf where a, b are each image resoluion and f the total
    #               number of frames.
    # known_maximums: list of known maximums on central frame in px, if maximums are unknown, empty list []
    #
    # OUTPUTS:
    # maxima_db:    maximums data base saved as dictionary with each point_id as key
    #               and location as value. frame_index can be retrived from the
    #               point_id -> frame_index = point_id//10000
    
    number_of_frames = video.shape[2]
    maxima_db = {'number_of_frames': number_of_frames}
    central_frame = number_of_frames//2
    
    for frame_index in range(number_of_frames):   #for each frame find local maxima and update database
        frame = video[:, :, frame_index]
        if(frame_index == central_frame):
            #set known maximums at baseline (if there are)
            if(len(known_maximums) > 0):
                point_key = frame_index*10000
                for knwon_maximum in known_maximums:
                    x = 39
                    y = knwon_maximum
                    maxima_db[point_key] = (x, y)
                    point_key += 1
            else:
                maxima_db = find_frame_local_maximums(np.copy(frame), frame_index, maxima_db)
        else:
            maxima_db = find_frame_local_maximums(np.copy(frame), frame_index, maxima_db)

    #set known maximums at baseline

    return maxima_db

def find_frame_local_maximums(frame, frame_index, maxima_db):
    # It returns a dictionary with each point_id as key and location as value. point_id:(x, y)
    # point_id is defined as xxyyyy where xx is the frame index and yyyy sequential number
    # the frame_index can be retrived from the point_id -> point_id//10000
    #
    # INPUTS:
    # frame:        image (numpy array) to be proceesed.
    # frame_index:  image index on the sewuence (video)
    # maxima_db:    maximums data base saved as dictionary
    #
    # OUTPUTS:
    # maxima_db:    updated maximums data base

    maximums_coordenates_proposal = peak_local_max(frame, min_distance = 1)

    resolution = (frame.shape[0] + 1)/2 - 1

    point_key = frame_index*10000
    for point in maximums_coordenates_proposal:
        x = point[0]
        y = point[1]

        #calculate distance from center
        d = np.sqrt((x - resolution)**2 + (y - resolution)**2)

        #do not consider maximums finded at the borders or outside the mask
        if (d < (resolution - 2)):
            maxima_db[point_key] = (int(x), int(y))
            point_key += 1

    return maxima_db

def find_basline_maximums(maximums_data_base):
    # Looks maximums located at the baseline on the central frame and returns a list with ids
    # INPUTS:
    # maximums_data_base:    maximums data base saved as dictionary with each point_id as key
    #               and location as value. frame_index can be retrived from the
    #               point_id -> frame_index = point_id//10000
    #
    # OUTPUTS:
    # baseline_poins_ids: list with ids from maximums located at the baseline

    central_frame = maximums_data_base['number_of_frames']//2

    baseline_poins_ids = []
    for point_id in maximums_data_base:
        if(type(point_id) == int):
            if(point_id//10000 == central_frame):
                x = maximums_data_base[point_id][0]
                y = maximums_data_base[point_id][1]

                #check if point is located on lef baseline
                if(x == 39 and y < 41):
                    baseline_poins_ids.append(point_id)

    # Here we can add image intensity filters to remove unwanted baseline maximums. These will
    # also require video as an input.
    # video:        images (or frames) sequence (numpy array) to be proceesed. Array shape
    #               must be axbxf where a, b are each image resoluion and f the total
    #               number of frames.

    return baseline_poins_ids


# -------------------- Trajectory analysis functions --------------------
def find_equidistances(maximums_data_base, config):
    # It looks for combinations of corresponding maximums on the maximums database and it
    # saves coincidences as pointers on a new databases.
    #
    # INPUTS:
    # maximums_data_base:    maximums data base saved as dictionary with each point_id as key
    #               and location as value. frame_index can be retrived from the
    #               point_id -> frame_index = point_id//10000
    #config :)
    # tolerance:    hola
    # maximum_distance_between_frames:  sets the maximum distance between frames to check. Its
    #               ok to saturate in order to avoid connecting far away unrrealted points.
    #
    # OUTPUTS:
    # [future_pointers_db, past_pointers_db]: Maximum pointers data base. Each data base its a
    #               dictionary where each key corresponds to a maximum point_id and its value
    #               to a list of ids where the key is pointing. In order to be able to apply
    #               temporal analysis 2 data bases are created, one with pointers going time-wise
    #               (fture) and the second one going counter-time-wise (past)


    maximum_distance_between_frames = config['equidistances_parameters']['maximum_distance_between_frames']
    tolerance = config['equidistances_parameters']['tolerance']

    future_pointers_db = {}
    past_pointers_db = {}

    number_of_frames = maximums_data_base['number_of_frames']

    for frame_index in range(number_of_frames):

        # Find de maximum posible distance btween frames
        maximum_delta_frames = min(number_of_frames - frame_index - 1, frame_index)

        # And saturate if necesary
        if(maximum_delta_frames > maximum_distance_between_frames):
            maximum_delta_frames = maximum_distance_between_frames

        for frames_distance in range(1, maximum_delta_frames + 1):
            future_pointers_db, past_pointers_db = equidistance_analysis(maximums_data_base, frame_index, frames_distance, future_pointers_db, past_pointers_db, tolerance)

    return [future_pointers_db, past_pointers_db]


def equidistance_analysis(maxima_db, frame_index, frames_distance, future_pointers_db, past_pointers_db, tolerance):
    # Compares 3 frames (past, present and future) with the same time distance and looks
    # for maximums that move the same way between the past-present frames and present-future frames.
    #
    # INPUTS:
    # maxima_db:    maximums data base saved as dictionary with each point_id as key
    #               and location as value. frame_index can be retrived from the
    #               point_id -> frame_index = point_id//10000
    #
    # frame_index:  central frame to analize (present frame)
    # 
    # frames_distance: distance beween the central frame and the two oder frames to analaize
    #
    # maximum_distance_between_frames:  sets the maximum distance between frames to check. Its
    #               ok to saturate in order to avoid connecting far away unrrealted points.
    # 
    # future_pointers_db: Pointers data base.
    # 
    # past_pointers_db: Pointers data base.
    # 
    # tolerance:    tolerance that defines if two the diference between two maximums posible
    #               trayectories can be considerer the same
    #
    #
    # OUTPUTS:
    # [future_pointers_db, past_pointers_db]: Updated pointers data base.

    max_distance = 4 + frames_distance #<-- arbitrarily define
    
    for id_1 in maxima_db:
        if(type(id_1) == int):
            if(id_1//10000 == frame_index):
                point_1 = maxima_db[id_1]
                
                points_distance = {}

                # calculate distance (on each coordenate) with each point form the previus image
                #calcular distancia (en cada coordenada) con cada punto de la imagen anterior
                for id_0 in maxima_db:
                    if(type(id_0) == int):
                        if(id_0//10000 == (frame_index - frames_distance)):
                            point_0 = maxima_db[id_0]
                            difference = (point_0[0] - point_1[0], point_0[1] - point_1[1])
                            if(abs(difference[0]) < max_distance and abs(difference[1]) < max_distance):
                                points_distance[id_0] = difference

                
                for id_2 in maxima_db:
                    if(type(id_2) == int):
                        if(id_2//10000 == (frame_index + frames_distance)):
                            point_2 = maxima_db[id_2]

                            for id_0 in points_distance:
                                #prepare data
                                d1 = points_distance[id_0]
                                dx1 = d1[0]
                                dy1 = d1[1]

                                dx2 = point_1[0] - point_2[0]
                                dy2 = point_1[1] - point_2[1]

                                #calculate difference on each axis
                                diff_x = abs(dx1 - dx2)
                                diff_y = abs(dy1 - dy2)

                                #if difference is to small ignore sign
                                signo_x_ok = (diff_x <= 1) or (np.sign(dx1) == np.sign(dx2))
                                signo_y_ok = (diff_y <= 1) or (np.sign(dy1) == np.sign(dy2))


                                #if difference makes sense, save pointer data              
                                if(signo_x_ok and (diff_x < tolerance) and signo_y_ok and (diff_y < tolerance)):
                                    future_pointers_db = update_poiters_db(future_pointers_db, id_0, id_1, id_2)
                                    past_pointers_db   = update_poiters_db(  past_pointers_db, id_2, id_1, id_0)

    return future_pointers_db, past_pointers_db


def update_poiters_db(pointers_db, id_0, id_1, id_2):
    if(id_0 in pointers_db):
        if(id_1 not in pointers_db[id_0]):
            pointers_db[id_0].append(id_1)
    else:
        pointers_db[id_0] = [id_1]

    if(id_1 in pointers_db):
        if(id_2 not in pointers_db[id_1]):
            pointers_db[id_1].append(id_2)
    else:
        pointers_db[id_1] = [id_2]
    
    return pointers_db


# -------------------- Trajectory trees functions --------------------
def look_for_trajectory_trees(baseline_poins_ids, future_pointers_db, past_pointers_db):
    trajectory_trees = {}

    for point_id in baseline_poins_ids:
        trajectories = []

        create_history_tree(point_id, future_pointers_db, [], trajectories)
        create_history_tree(point_id,   past_pointers_db, [], trajectories)

        trajectory_trees[point_id] = trajectories.copy()

    return trajectory_trees

def create_history_tree(pointer, pointers, history_tree, trajectories_history):

    history = history_tree.copy()
    
    history.append(pointer)
    while 1:
        if(pointer in pointers):
            pointer_data = pointers[pointer]
            #print(pointer, '->', pointer_data)
            if(len(pointer_data) > 1):
                for next_pointer in pointer_data:
                    create_history_tree(next_pointer, pointers, history, trajectories_history)
                break
            else:
                pointer = pointer_data[0]
                history.append(pointer)
        else:
            trajectories_history.append(history.copy())
            break

# -------------------- Trajectory filtering functions --------------------
def filter_trajectories(trajectories_db, points_db, config):

    filtered_trajectories = {}
    
    for baseline_point_id in trajectories_db:
        filter_1 = filter_curved_branches(trajectories_db[baseline_point_id].copy(), points_db, config['trajectories_filter']['max_angle'])

        filter_2 = filter_momenta(filter_1.copy(), points_db, config['trajectories_filter']['momentum_tolerance'])

        filter_3 = filter_thin_branches(filter_2.copy(), config['trajectories_filter']['thin_percent'])

        filter_4 = filter_short_branches(filter_3.copy(), config['trajectories_filter']['short_percent'])

        filtered_trajectories[baseline_point_id] = filter_4.copy()

    return filtered_trajectories

def filter_curved_branches(trajectories_input, points_db, max_angle):
    # Check the angle between each each group of three points. If the
    # angle is grater than max_angle, that branch is removed.
    #
    # INPUTS:
    # trajectories_input:   Group of trajectories to filter
    # points_db:            Maximums points data base
    # max_angle:            Maximum angle allowed on branches
    #
    # OUTPUTS:
    # trajectories_output:  Filtered trajectories

    trajectories_output = []
    for trajectory in trajectories_input:
    
        #Use grups of 3 points
        ok = True
        for point_id in range(len(trajectory) - 2):

            id_0 = trajectory[point_id + 0]
            id_1 = trajectory[point_id + 1]
            id_2 = trajectory[point_id + 2]

            p_0 = np.array(points_db[id_0])
            p_1 = np.array(points_db[id_1])
            p_2 = np.array(points_db[id_2])

            d_1 = np.linalg.norm(p_0 - p_1)     #distance p0 - p1
            d_2 = np.linalg.norm(p_1 - p_2)     #distance p1 - p2

            if(d_1 >= 1 and d_2 >= 1):
                A = p_1 - p_2
                B = p_1 - p_0
                
                if(abs(A[1]) > 0 and abs(B[1]) > 0):
                    m1 = A[0]/A[1]
                    m2 = B[0]/B[1]
                elif(abs(A[0]) > 0 and abs(B[0]) > 0):
                    m1 = A[1]/A[0]
                    m2 = B[1]/B[0]
                else:
                    m1 = 1
                    m2 = 0

                if(m1 != m2):
                    delta_angle = np.degrees(np.arccos(np.dot(A, B)/(np.linalg.norm(A) * np.linalg.norm(B))))
                    
                    angle = abs(delta_angle)
                else:
                    angle = 180

                if(angle < max_angle):
                    ok = False

        if(ok):
            trajectories_output.append(trajectory)

    return trajectories_output.copy()

def filter_momenta(trajectories_input, points_db, tolerance):
    trajectories_output = []
    for trajectory in trajectories_input:
        ok = True

        momenta_x = []
        momenta_y = []

        for first_point_id in range(len(trajectory) - 1):
            id_0 = trajectory[first_point_id + 0]
            id_1 = trajectory[first_point_id + 1]

            frame_0 = id_0//10000
            frame_1 = id_1//10000

            delta_frames = frame_0 - frame_1

            dx = (points_db[id_0][0] - points_db[id_1][0])/delta_frames
            momenta_x.append(dx)

            dy = (points_db[id_0][1] - points_db[id_1][1])/delta_frames
            momenta_y.append(dy)

        if(len(momenta_x) == 0 or len(momenta_y) == 0):
            ok = False
        else:
            mean_x = sum(momenta_x)/len(momenta_x)
            mean_y = sum(momenta_y)/len(momenta_y)

            ok = True
            for dx in momenta_x:
                if abs(dx - mean_x) > tolerance:
                    ok = False

            for dy in momenta_y:
                if abs(dy - mean_y) > tolerance:
                    ok = False

        if(ok):
            trajectories_output.append(trajectory.copy())

    return trajectories_output.copy()

def filter_thin_branches(trajectories_input, filter_percent):
    # Filters thin branches
    # INPUTS:
    # trajectories_input:   Group of trajectories to filter
    # filter_percent:       Miminum filter percent
    #
    # OUTPUTS:
    # trajectories_output:  Filtered trajectories


    points = {}
    for trajectory in trajectories_input:
        final_point = trajectory[-1]
        if(final_point in points):
            points[final_point] += 1
        else:
            points[final_point] = 1
    
    maximum = 0
    for point_id in points:
        quantity = points[point_id]
        if(quantity > maximum):
            maximum = quantity

    #todas las ramas que tienen menos del prociento_filtro de ocurrencia que la maxima para afuera
    ok_points = []
    for point_id in points:
        quantity = points[point_id]
        if(quantity/maximum > filter_percent):
            ok_points.append(point_id)

    #cortamos las ramas
    trajectories_output = []
    for trajectory in trajectories_input: 
        if(trajectory[-1] in ok_points):
            trajectories_output.append(trajectory)

    return trajectories_output.copy()

def filter_short_branches(trajectories_input, short_percent):
    # Filters shoert branches
    # INPUTS:
    # trajectories_input:   Group of trajectories to filter
    # filter_percent:       Miminum filter percent
    #
    # OUTPUTS:
    # trajectories_output:  Filtered trajectories

    quantity = []
    for trajectory in trajectories_input:
        quantity.append(len(trajectory))

    quantity = [x / max(quantity) for x in quantity]
    valid = [x > short_percent for x in quantity]
    

    trajectories_output = []
    for inice_trajectory in range(len(trajectories_input)):
        if(valid[inice_trajectory]):
            trajectories_output.append(trajectories_input[inice_trajectory].copy())
    
    return trajectories_output.copy()

    



# --------------- Trajectories merge ----------
def estimate_and_merge_trajectories(filtered_trajectories, maximums_data_base, config):
    tolerancia = config['merge_tolerance']
    # Estima rectas y junta las parecidas
    # tolerancia en grados!

    #1 Juntamos puntos de rectas con mismo inicio y fin
    resultados = {}
    for id_cruce in filtered_trajectories:
        if(id_cruce not in resultados):
            resultados[id_cruce] = {}
        
        for trayectoria in filtered_trajectories[id_cruce]:
            punto_inicial = trayectoria[0]
            punto_final = trayectoria[-1]

            llave = (punto_inicial, punto_final)
            
            if(llave not in resultados[id_cruce]):
                resultados[id_cruce][llave] = []

            for punto in trayectoria:
                if(punto not in resultados[id_cruce][llave]):
                    resultados[id_cruce][llave].append(punto)

    # 2 Calcular dx y dy de cada caso
    velocidades_resultados = {}
    for cruce in resultados:
        velocidades_resultados[cruce] = {}
        for llave in resultados[cruce]:
            trayectoria = resultados[cruce][llave]
            dx, dy = calculate_velocity(trayectoria.copy(), maximums_data_base)
            if not (abs(dx) < 0.000001 and abs(dy) < 0.000001):
                velocidades_resultados[cruce][llave] = (dx, dy)

    resultados_2 = {}
    velocidades_resultados_2 = {}
    # 3 juntar grupos de casos demsiado parecidos en cada cruce
    for id_cruce in velocidades_resultados:
        if(len(velocidades_resultados[id_cruce]) == 1):
            grupos = {}
            for llave in velocidades_resultados[id_cruce]:
                grupos[len(grupos)] = [llave]

        elif(len(velocidades_resultados[id_cruce]) > 1):
            pares = []
            pares_validos = []
            contador_i = 0
            for llave_i in velocidades_resultados[id_cruce]:
                contador_j = 0
                for llave_j in velocidades_resultados[id_cruce]:
                    if(contador_i < contador_j):
                        v1 = velocidades_resultados[id_cruce][llave_i]
                        v2 = velocidades_resultados[id_cruce][llave_j]

                        delta_angulo = calculate_angle_between_vectors(v1, v2)

                        if(delta_angulo < tolerancia):
                            pares.append([(llave_i, llave_j), delta_angulo])
                            pares_validos.append((llave_i, llave_j))

                    contador_j += 1
                contador_i += 1


            if(len(pares) > 1):
                # 1 Ordenar por distancia
                sorted_list = sorted(pares, key=lambda x : x[1])
                
                # 2 armar grupo segun distancias
                grupos = {}
                grupos[0] = [sorted_list[0][0][0], sorted_list[0][0][1]]


                for indice_menor in range(1, len(sorted_list)):
                    trayectoria_1 = sorted_list[indice_menor][0][0]
                    trayectoria_2 = sorted_list[indice_menor][0][1]

                    grupo_minimo = -1
                    distancia_total_grupo_minimo = 1000000
                    for grupo in grupos:
                        
                        ok_total = True
                        distancia_total = 1000000
                        #esta dentro del rango con todos los elementos del grupo
                        for trayectoria in grupos[grupo]:
                            
                            ok_1 = False
                            distancia_1 = 1000000
                            if(trayectoria == trayectoria_1):
                                ok_1 = True
                                distancia_1 = 0
                            else:
                                if((trayectoria, trayectoria_1) in pares_validos or (trayectoria_1, trayectoria) in pares_validos):
                                    ok_1 = True
                                    if((trayectoria, trayectoria_1) in pares_validos):
                                        for par in pares:
                                            if par[0] == (trayectoria, trayectoria_1):
                                                distancia_1 = par[1]

                                    if((trayectoria_1, trayectoria) in pares_validos):
                                        for par in pares:
                                            if par[0] == (trayectoria_1, trayectoria):
                                                distancia_1 = par[1]
                            
                            ok_2 = False
                            distancia_2 = 1000000
                            if(trayectoria == trayectoria_2):
                                ok_2 = True
                                distancia_2 = 0
                            else:
                                if((trayectoria, trayectoria_2) in pares_validos or (trayectoria_2, trayectoria) in pares_validos):
                                    ok_2 = True
                                    if((trayectoria, trayectoria_2) in pares_validos):
                                        for par in pares:
                                            if par[0] == (trayectoria, trayectoria_2):
                                                distancia_2 = par[1]

                                    if((trayectoria_2, trayectoria) in pares_validos):
                                        for par in pares:
                                            if par[0] == (trayectoria_2, trayectoria):
                                                distancia_2 = par[1]

                            if(not ok_1 or not ok_2):
                                ok_total = False

                            d_12 = max([distancia_1, distancia_2])
                            if(d_12 < distancia_total):
                                distancia_total = d_12
                                grupo_minimo = grupo
                        
                        if(ok_total):
                            if(distancia_total < distancia_total_grupo_minimo):
                                distancia_total_grupo_minimo = distancia_total

                    if(grupo_minimo != -1):
                        #Agregarlos solo si ambos no estan ya en otro grupo!!!!
                        esta_en_otro_grupo_1 = False
                        for grupo_2 in grupos:
                            if(grupo_2 != grupo):
                                if(trayectoria_1 in grupos[grupo_2]):
                                    esta_en_otro_grupo_1 = True

                        if(not esta_en_otro_grupo_1):           
                            if(trayectoria_1 not in grupos[grupo]):
                                grupos[grupo].append(trayectoria_1)

                        esta_en_otro_grupo_2 = False
                        for grupo_2 in grupos:
                            if(grupo_2 != grupo):
                                if(trayectoria_1 in grupos[grupo_2]):
                                    esta_en_otro_grupo_2 = True

                        if(not esta_en_otro_grupo_2):           
                            if(trayectoria_2 not in grupos[grupo]):
                                grupos[grupo].append(trayectoria_2)

                    else:
                        grupos[len(grupos)] = [trayectoria_1, trayectoria_2]

            elif(len(pares) == 1):
                #1 Ordenar por distancia
                sorted_list = sorted(pares, key=lambda x : x[1])

                # 2 armar grupo segun distancias
                grupos = {}
                grupos[0] = [sorted_list[0][0][0], sorted_list[0][0][1]]

            else:
                grupos = {}
                for llave in velocidades_resultados[id_cruce]:
                    grupos[len(grupos)] = [llave]
            
        else:
            grupos = {}

        # 4 Juntar puntos de cada grupo formado
        resultados_2[id_cruce] = {}
        for grupo in grupos:
            resultados_2[id_cruce][grupo] = []
            for trayectoria in grupos[grupo]:
                for punto in resultados[id_cruce][trayectoria]:
                
                    if(punto not in resultados_2[id_cruce][grupo]):
                        resultados_2[id_cruce][grupo].append(punto)

        #5 calcular velocidades de los nuevos gurpos
        velocidades_resultados_2[id_cruce] ={}
        for id_trayectoria in resultados_2[id_cruce]:
            dx, dy = calculate_velocity(resultados_2[id_cruce][id_trayectoria].copy(), maximums_data_base)
            if not (abs(dx) < 0.000001 and abs(dy) < 0.000001):
                px = maximums_data_base[id_cruce][1]
                m_px = config['tel_parameters']['height_m_px']
                h_est_m = round((39 - px)*m_px)

                frames_sample = 1/config['correlation']['fpi']
                samples_seg = config['tel_parameters']['samples_seg']

                if(config['tel_parameters']['lgh_m'] != 0):
                    const_h = (m_px*frames_sample*samples_seg/1500)/(1 + h_est_m/config['tel_parameters']['lgh_m'])
                else:
                    const_h = (m_px*frames_sample*samples_seg/1500)

                
                dx_m_s = dx*const_h
                dy_m_s = dy*const_h

                velocidades_resultados_2[id_cruce][id_trayectoria] = (h_est_m, -dy_m_s, dx_m_s)


    # 5 Preparar datos finales
    resultados = []
    for id_cruce in velocidades_resultados_2:
        if(len(velocidades_resultados_2[id_cruce]) > 0):
            h_est_m = velocidades_resultados_2[id_cruce][0][0]

            for id_trayectoria in velocidades_resultados_2[id_cruce]:
                dx = velocidades_resultados_2[id_cruce][id_trayectoria][1]
                dy = velocidades_resultados_2[id_cruce][id_trayectoria][2]
                resultados.append((h_est_m, dx, dy))

    return resultados

def simple_linear_regression(x_fit, y_fit):
    # Calculates simple linear regression and returns y = ax + b
    # INPUTS:
    # x_fit, y_fit: Lists with x and y data points respectively
    #
    # OUTPUTS:
    # a, b:         Constants who describe fitted line y = ax + b 

    sum_x = 0
    sum_y = 0
    sum_xx = 0
    sum_yy = 0
    sum_xy = 0
    n = 0

    for point_index in range(len(x_fit)):
        x = x_fit[point_index]
        y = y_fit[point_index]

        sum_x += x
        sum_y += y
        sum_xx += x*x
        sum_yy += y*y
        sum_xy += x*y
        n += 1

    if((n*sum_xx - sum_x*sum_x) > 0):
        b = (n*sum_xy - sum_x*sum_y)/(n*sum_xx - sum_x*sum_x)
        a = (sum_y - b*sum_x)/n
    else:
        print('Error on linear regression!')
        a = -1
        b = -1

    return a, b

def calculate_velocity(points, maximums_data_base):
    # Calculates diferential pixels on each axis of the trajectory
    #
    # INPUTS:
    # points:       Trayectory points
    #
    # maximums_data_base:    maximums data base saved as dictionary with each point_id as key
    #               and location as value. frame_index can be retrived from the
    #               point_id -> frame_index = point_id//10000
    #
    # OUTPUTS:
    # dx, dy:       differential (delta) pixels on each axis

    # 1 Prepare space for each frame
    points_dt = {}
    for tic in range(maximums_data_base['number_of_frames'] + 1):
        points_dt[tic] = []
    
    # 2 Save each point data at its timestamp
    for point in points:
        tic = point//10000
        points_dt[tic].append(point)
    
    vx = []
    vy = []

    # 3 Get cooredenates and meaon on each timestamp
    for tic in points_dt:
        if(len(points_dt[tic]) > 0):
            px = [maximums_data_base[point][0] for point in points_dt[tic]]
            py = [maximums_data_base[point][1] for point in points_dt[tic]]

            mean_x = sum(px)/len(px)
            mean_y = sum(py)/len(py)

            vx.append((mean_x, tic))
            vy.append((mean_y, tic))

    # 4 Calculate dx y dy by fitting distance through time on each axis
    vx_x = [p[1] for p in vx]
    vx_y = [p[0] for p in vx]
    n, dx = simple_linear_regression(vx_x, vx_y)

    vy_x = [p[1] for p in vy]
    vy_y = [p[0] for p in vy]
    n, dy = simple_linear_regression(vy_x, vy_y)
            
    return dx, dy    

def calculate_angle_between_vectors(v1, v2):
    # Calculates angle between two 2D vectors
    #
    # INPUTS:
    # v1, v2:   Vector inputs (as lists or arrays)
    #
    # OUTPUTS:
    # angle:    Angle between v1 and v2

    dx_1, dy_1 = v1
    dx_2, dy_2 = v2

    # Normalize vectors
    norm_1 = np.sqrt(dx_1**2 + dy_1**2)
    dx_1_n = dx_1/norm_1
    dy_1_n = dy_1/norm_1

    norm_2 = np.sqrt(dx_2**2 + dy_2**2)
    dx_2_n = dx_2/norm_2
    dy_2_n = dy_2/norm_2

    # Calculate angle
    try:
        angle = np.rad2deg(np.arccos(np.dot(np.array([dx_1_n, dy_1_n]), np.array([dx_2_n, dy_2_n]))))
    except:
        print('Error in angle between vectors', v1, v2)

    return angle


def IPTA(video, config, layesr_height_m):

    layesr_height_px = [round(39 - h/config['tel_parameters']['height_m_px']) for h in layesr_height_m]

    maximums_data_base = find_video_local_maximums(video, layesr_height_px)
    baseline_poins_ids = find_basline_maximums(maximums_data_base) # <- cambiar esto en caso de tener maximos conocidos (agregarlos en caso de ser necesario)

    [future_pointers_db, past_pointers_db] = find_equidistances(maximums_data_base, config)
    
    trajectories = look_for_trajectory_trees(baseline_poins_ids, future_pointers_db, past_pointers_db)
    
    filtered_trajectories = filter_trajectories(trajectories, maximums_data_base, config)

    estimated_trajectories = estimate_and_merge_trajectories(filtered_trajectories, maximums_data_base, config)

    return estimated_trajectories
