

scan = [[0, 100], [0.1, 101], [0.3, 105], [0.4, 97],
        [1, 80], [1.5, 78], [1.6, 85], [1.9, 81],
        [3, 9], [3.1, 12], [3.7, 8], [3.8, 10],
        [6, 40], [6.7, 39]
        ]

deg_view = [0] * 360
deg_data_cnt = 0
last_deg = -1000
insert_list = []
for data in scan:
    deg = data[0]
    dist = data[1]

     # deg as index            
    deg = int(deg)

    # new scan, update deg
    if last_deg == -1000:
        last_deg = deg
    # new data, calculate last deg
    elif last_deg != deg:
        deg_view[last_deg] /= deg_data_cnt
        deg_data_cnt = 0
        # need insert
        if deg - last_deg != 1:
            insert_list.append([last_deg, deg])
        last_deg = deg
        
    # count data in this deg
    deg_data_cnt += 1
    
    # first data in this deg
    if deg_data_cnt == 1:
        deg_view[deg] = dist
    # other sample in this deg
    else:
        deg_view[deg] += dist

    # last data, calculate current deg
    if data == scan[-1]:
        deg_view[deg] /= deg_data_cnt

# insert process
for insert in insert_list:
    delta_deg = insert[1] - insert[0]
    dist_step = (deg_view[insert[1]] - deg_view[insert[0]]) / delta_deg
    for deg in range(insert[0]+1, insert[1]):
        deg_view[deg] = deg_view[deg-1] + dist_step

print(deg_view)