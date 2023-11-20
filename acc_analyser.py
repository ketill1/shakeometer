import matplotlib.pyplot as plt
import yaml
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
from scipy.signal import stft

class Data:

    def __init__(self):
        self.arrival_time = []
        self.acc = []

        self.zero_time = 0
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
                    self.arrival_time.append(sec + nanosec)
                    # a = msg.linear_acceleration.x + msg.linear_acceleration.y + msg.linear_acceleration.z
                    a = msg.linear_acceleration.z
                    self.acc.append(a)
            
            self.arrival_time = np.array(self.arrival_time)
            self.acc = np.array(self.acc)

    def fft(self, acc_z):
        # FFT
        N = len(acc_z)

        fft_result = np.fft.fft(acc_z)
        fft_freq = np.fft.fftfreq(N, d=1./self.sampling_rate)

        # Taking the absolute value to get magnitude
        fft_magnitude = np.abs(fft_result)
        return fft_magnitude, fft_freq
    
    def calculate_rms(self):
        return np.sqrt(np.mean(np.square(self.acc)))
    
    def calculate_stft(self):
        f, t, Zxx = stft(self.acc, fs=self.sampling_rate)
        return f, t, np.abs(Zxx)

    def plot_data(self, data):

        fig, axs = plt.subplots(nrows=3, ncols=1, figsize=[12,10])
        
        fft_magnitude, fft_freq = data.fft(data.acc)

        # Filter to avoid the spike at 0 Hz and focus around half the maximum frequency
        lower_bound = 0.1  # Start slightly above 0 to avoid the spike
        upper_bound = self.sampling_rate / 4  # Approximately half the Nyquist frequency
        valid_freqs = (fft_freq > lower_bound) & (fft_freq < upper_bound)

        fft_magnitude = fft_magnitude[valid_freqs]
        fft_freq = fft_freq[valid_freqs]

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

        f, t, Zxx = data.calculate_stft()
        im = axs[2].pcolormesh(t, f, np.abs(Zxx), shading='gouraud')
        axs[2].set_title('STFT Magnitude')
        axs[2].set_xlabel('Time (sec)')
        axs[2].set_ylabel('Frequency (Hz)')
        axs[2].set_ylim([0, 30])
        axs[2].grid()
        fig.colorbar(im, ax=axs[2])
        axs[2].grid()

        plt.tight_layout()  # Adjusts the plots to fit into the figure area.
        plt.show()
        plt.clf()

        print(self.sampling_rate)
        print(data.calculate_rms())



def main():
    data = Data()
    data.find_sample_rate()
    data.get_data()
    data.plot_data(data)
    

if __name__ == '__main__':
    main()