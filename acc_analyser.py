import matplotlib.pyplot as plt
import yaml
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
from scipy.signal import butter, filtfilt, stft, welch, find_peaks

import os

class Data:

    def __init__(self, bag):
        self.arrival_time = []
        self.acc = []

        self.zero_time = 0
        self.sampling_rate = 0 # Hz

        self.bag_path = 'rosbag'
        self.bag = bag

        self.topic = '/imu/data/hr'

    def find_sample_rate(self):
        with open(self.bag_path + '/' + self.bag + '/metadata.yaml') as file:
            acc_yaml = yaml.load(file, Loader=yaml.FullLoader)
            durration = acc_yaml['rosbag2_bagfile_information']['duration']['nanoseconds']
            message_count = acc_yaml['rosbag2_bagfile_information']['message_count']
            self.sampling_rate = message_count / (durration * 10 ** -9)


    def get_data(self):
        with Reader('./' + self.bag_path + '/' + self.bag) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == self.topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    if self.zero_time == 0:
                        self.zero_time = msg.header.stamp.sec

                    sec = msg.header.stamp.sec - self.zero_time
                    nanosec = msg.header.stamp.nanosec * 10 ** -9
                    self.arrival_time.append(sec + nanosec)
                    self.acc.append((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
            
            self.arrival_time = np.array(self.arrival_time)
            self.acc = np.array(self.acc)
    
    def butter_highpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a


    def apply_highpass_filter(self, cutoff_frequency):
        b, a = self.butter_highpass(cutoff_frequency, self.sampling_rate)
        self.acc[:,2] = filtfilt(b, a, self.acc[:, 2])

    def remove_gravity(self):
        # Remove gravity
        mean_acc_z = np.mean(self.acc[:, 2])
        print(f"Mean acc z: {mean_acc_z}")
        self.acc[:, 2] = self.acc[:, 2] - mean_acc_z

    def fft(self, acc):
        # FFT
        N = len(acc)

        fft_result = np.fft.fft(acc)
        fft_freq = np.fft.fftfreq(N, d=1./self.sampling_rate)

        # Taking the absolute value to get magnitude
        fft_magnitude = np.abs(fft_result)
        return fft_magnitude, fft_freq
    
    def calculate_rms(self):
        return np.sqrt(np.mean(np.square(self.acc[:, 2])))
    
    def calculate_stft(self):
        f, t, Zxx = stft(self.acc[:, 2], fs=self.sampling_rate)
        return f, t, np.abs(Zxx)
    
    def calculate_psd(self):
        f, Pxx = welch(self.acc[:, 2], fs=self.sampling_rate, nperseg=1024)
        return f, Pxx
    
    def find_peaks_in_psd(self, Pxx, height=None):
        peaks, _ = find_peaks(Pxx, height=height)
        return peaks
        
    def plot_data(self, data):
        fig, axs = plt.subplots(nrows=2, ncols=1, figsize=[12,10])
        cmap = plt.get_cmap('viridis')
        n_dim = data.acc.shape[1]
        colors = [cmap(i) for i in np.linspace(0, 1, n_dim)]
        for i in range(n_dim):
            fft_magnitude, fft_freq = data.fft(data.acc[:, i])
            # Filter to avoid the spike at 0 Hz and focus around half the maximum frequency
            lower_bound = 0.0  # Start slightly above 0 to avoid the spike
            upper_bound = self.sampling_rate / 2  # Approximately the Nyquist frequency
            valid_freqs = (fft_freq > lower_bound) & (fft_freq < upper_bound)

            fft_magnitude = fft_magnitude[valid_freqs]
            fft_freq = fft_freq[valid_freqs]

            # Plot the FFT result in the first subplot
            axs[0].plot(fft_freq, fft_magnitude, color=colors[n_dim-1-i], linewidth=1)
            axs[1].plot(data.arrival_time, data.acc[:, i], color=colors[n_dim-1-i], linewidth=1)

        axs[0].set_title('Frequency Spectrum')
        axs[0].set_xlabel('Frequency (Hz)')
        axs[0].set_ylabel('Magnitude')
        axs[0].grid()

        # Plot the time series data in the second subplot
        axs[1].set_title('Acceleration Time Series')
        axs[1].set_xlabel('Time (sec)')
        axs[1].set_ylabel('Acceleration (z-axis)')
        axs[1].grid()

        fig1, axs1 = plt.subplots(nrows=2, ncols=1, figsize=[12,10])

        f, t, Zxx = data.calculate_stft()
        im = axs1[0].pcolormesh(t, f, np.abs(Zxx), shading='gouraud')
        # Change the limits of the plot to only show the data we care about
        axs1[0].set_ylim([0, self.sampling_rate / 4])
        axs1[0].set_title('STFT Magnitude')
        axs1[0].set_xlabel('Time (sec)')
        axs1[0].set_ylabel('Frequency (Hz)')
        axs1[0].grid()
        fig1.colorbar(im, ax=axs1[0])
        axs1[0].grid()

        # PSD Plot
        f_psd, Pxx = data.calculate_psd()
        peaks = data.find_peaks_in_psd(Pxx, height=1)  # limit the lowest peak.

        axs1[1].plot(f_psd, Pxx)
        axs1[1].plot(f_psd[peaks], Pxx[peaks], "x")
        # axs1[1].set_xlim([0, 30])
        axs1[1].set_title('Power Spectral Density')
        axs1[1].set_xlabel('Frequency (Hz)')
        axs1[1].set_ylabel('PSD [V**2/Hz]')


        plt.tight_layout()  # Adjusts the plots to fit into the figure area.
        plt.show()
        plt.clf()


def main():
    parent_directory = './rosbag'  # Replace with the path to your parent directory

    # List all directories in the parent directory
    available_folders = [d for d in os.listdir(parent_directory) if os.path.isdir(os.path.join(parent_directory, d))]

    # Check if there are available folders
    if not available_folders:
        print("No data folders found.")
        return

    # Print the menu
    print("Select a file to analyze:")
    for i, (bag_name) in enumerate(available_folders):
        print(f"{i + 1}: {bag_name}")

    # User input for selection
    choice = int(input("Enter your choice (number): ")) - 1

    # Get the selected file
    selected_bag = available_folders[choice]

    # Create an instance of Data with the selected file
    data = Data(selected_bag)
    data.find_sample_rate()
    data.get_data()
    # data.apply_highpass_filter(cutoff_frequency=0.1)
    data.remove_gravity()
    data.plot_data(data)

    print(f"Sampling Rate: {data.sampling_rate}")
    print(f"Zero Time: {data.zero_time}")
    print(f"RMS: {data.calculate_rms()}")
    print(f"Arrival Time: {data.arrival_time[:100]}")

if __name__ == '__main__':
    main()