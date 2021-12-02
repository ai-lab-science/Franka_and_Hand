import numpy as np
from matplotlib import pyplot as plt
from scipy.fft import rfft,irfft,rfftfreq


def plot_analysis(tactile_data,finger_name,axis_name,fig,ax,threshold=None):
    """
    Plots the raw data from the tactile sensors and the filtered data i.e. after applying Fast Fourier Transform(FFT).
    The plot consists of the raw and filtered data in frequency as well as time domain.
    Basically, that data is converted from Time domain to Frequency domain via FFT, then filtered. The filtering is
    done through a threshold value that restricts the certain frequency range. It is to be selected by the user. After
    the thresholding is done, the data is filtered back from Frequency domain to Time domain via Inverse FFT.

    Parameters
    ----------
    :param list tactile_data: Raw Tactile data for one particular axis from the Sensors in Time Domain.
    :param str finger_name: Name of the finger whose data is to be displayed.
    :param str axis_name: The name of the axis for window title.
    :param float threshold: The threshold value for frequency. The data with frequency above this threshold
                            is filtered out(Default=None).
    """
    # Select Threshold value
    if threshold is None:
        threshold = 0.250

    # Setting up Plot axis and conversion to frequency domain via FFT
    sampling_rate = 40
    time_axis = np.linspace(0,len(tactile_data),len(tactile_data))    # x-axis for time
    time_axis.sort()
    tactile_data_fft = rfft(tactile_data)    # y-axis for tactile data in frequency domain
    freq = rfftfreq(len(tactile_data), 1/sampling_rate)

    # Plot the raw tactile data
    fig.canvas.manager.set_window_title(finger_name + '_' + axis_name)
    ax[0, 0].plot(freq, np.abs(tactile_data_fft))
    ax[0, 0].set_title("Frequency domain before filtering")
    ax[0, 1].plot(time_axis, tactile_data)
    ax[0, 1].set_title("Time domain before filtering")

    # Filtering data with frequency threshold
    idx = np.where(freq>threshold)[0]
    tactile_data_fft[idx[0]:idx[-1]] = 0
    tactile_data_fft[idx[-1]] = 0

    # Converting data to time domain via Inverse FFT
    tactile_data_inv_fft = irfft(tactile_data_fft,n=len(time_axis))

    # Plot the filtered tactile data
    ax[1,0].plot(freq, np.abs(tactile_data_fft))
    ax[1, 0].set_title("Frequency domain after filtering")
    ax[1,0].axvline(x=threshold,c='r')
    ax[1,1].plot(time_axis,tactile_data_inv_fft)
    ax[1, 1].set_title("Time domain after filtering")