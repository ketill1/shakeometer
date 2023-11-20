# loading in modules
# import sqlite3
import csv
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import yaml
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import math


from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

class Data:

    def __init__(self):
        self.arrival_time = np.array([])
        self.zero_time = 0

        self.acc = np.array([])

        self.sampling_rate = 0 # Hz

    def find_sample_rate(self):
        with open(r'rosbag2/rosbag2_2023_11_20-20_50_28/metadata.yaml') as file:
            acc_yaml = yaml.load(file, Loader=yaml.FullLoader)
            durration = acc_yaml['rosbag2_bagfile_information']['duration']['nanoseconds']
            message_count = acc_yaml['rosbag2_bagfile_information']['message_count']
            self.sampling_rate = message_count / (durration * 10 ** -9)
            print(self.sampling_rate)


    def get_data(self):

        with Reader("./rosbag2/rosbag2_2023_11_20-20_50_28") as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/imu/data/hr':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    if self.zero_time == 0:
                        self.zero_time = msg.header.stamp.sec

                    sec = msg.header.stamp.sec - self.zero_time
                    nanosec = msg.header.stamp.nanosec * 10 ** -9
                    self.arrival_time= np.append(self.arrival_time, sec + nanosec)
                    # a = msg.linear_acceleration.x + msg.linear_acceleration.y + msg.linear_acceleration.z
                    a = msg.linear_acceleration.z
                    self.acc = np.append(self.acc, a)

    def fft(self, acc_z):
        # FFT
        N = len(acc_z)

        fft_result = np.fft.fft(acc_z)
        fft_freq = np.fft.fftfreq(N, d=1./self.sampling_rate)

        # Taking the absolute value to get magnitude
        fft_magnitude = np.abs(fft_result)
        return fft_magnitude, fft_freq




def main():
    fig, axs = plt.subplots(nrows=1, ncols=2, figsize=[12,10])

    data = Data()
    data.find_sample_rate()
    data.get_data()
    if 1 == 1:
        fft_magnitude, fft_freq = data.fft(data.acc)
        pos_freqs = fft_freq >= 0
        fft_magnitude = fft_magnitude[pos_freqs]
        fft_freq = fft_freq[pos_freqs]

        # Plot the FFT result in the first subplot
        axs[0].plot(fft_freq, fft_magnitude, 'b')
        axs[0].set_title('Frequency Spectrum')
        axs[0].set_xlabel('Frequency (Hz)')
        axs[0].set_ylabel('Magnitude')
        axs[0].grid()

    # Plot the time series data in the second subplot
    axs[1].plot(data.arrival_time, data.acc, 'r')
    axs[1].set_title('Acceleration Time Series')
    axs[1].set_xlabel('Time (sec)')
    axs[1].set_ylabel('Acceleration (z-axis)')
    axs[1].grid()

    plt.tight_layout()  # Adjusts the plots to fit into the figure area.
    plt.show()
    plt.clf()

if __name__ == '__main__':
    main()