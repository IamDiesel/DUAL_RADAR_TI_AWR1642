# basic cfar ca algorithmen from https://pypi.org/project/pyAPRiL/
import numpy as np
#from Plotter import size_range, size_doppler

#size_range = Plotter.size_range
#size_doppler = Plotter.size_doppler

def CA_CFAR_CALL(rd_matrix, size_range, size_doppler):
    #global size_range, size_doppler
    params = [4, 8, 2, 4]
    thresh = 4.0

    s = (size_range, size_doppler)
    ar = np.zeros(s)
    for rn in range(0,size_range):
        #rd_matrix[rn] = np.fft.fftshift(rd_matrix[rn])
        ar[rn] = rd_matrix[rn]
    
    cfar_list = CA_CFAR_naive(ar, params, thresh)

    return cfar_list

def CA_CFAR_naive(rd_matrix, win_param, threshold):
    # -- Set inital parameters --

    win_len = win_param[0]
    win_width = win_param[1]
    guard_len = win_param[2]
    guard_width = win_param[3]

    norc = np.size(rd_matrix, 1)  # number of range cells
    noDc = np.size(rd_matrix, 0)  # number of Doppler cells
    hit_matrix = []

    # Convert range-Doppler map values to power
    rd_matrix = np.abs(rd_matrix) ** 2

    # Generate window mask
    rd_block = np.zeros((2 * win_width + 1, 2 * win_len + 1), dtype=float)
    mask = np.ones((2 * win_width + 1, 2 * win_len + 1))
    mask[win_width - guard_width:win_width + 1 + guard_width, win_len - guard_len:win_len + 1 + guard_len] = np.zeros(
        (guard_width * 2 + 1, guard_len * 2 + 1))

    cell_counter = np.sum(mask)

    # Convert threshold value
    threshold = 10 ** (threshold / 10) 
    threshold /= cell_counter

    # -- Perform automatic detection --
    for j in np.arange(win_width, noDc - win_width, 1):  # Range loop
        for i in np.arange(win_len, norc - win_len, 1):  # Doppler loop
            rd_block = rd_matrix[j - win_width:j +
                                 win_width + 1, i - win_len:i + win_len + 1]
            rd_block = np.multiply(rd_block, mask)
            cell_SINR = rd_matrix[j, i] / np.sum(rd_block)  # esimtate CUT SINR

            # Hard decision           
            if cell_SINR > threshold:
                hit_matrix.append((j,i,rd_matrix[j][i]))

    return hit_matrix