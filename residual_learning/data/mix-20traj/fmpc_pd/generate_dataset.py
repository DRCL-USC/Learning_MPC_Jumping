import numpy as np
import pickle
import os

directory = os.path.dirname(os.getcwd())
print(f'directory = {directory}')

def save_to_pickle(measurements, file_path):
    # Save the measurements to a pickle file
    with open(file_path, 'wb') as f:
        pickle.dump(measurements, f)

if __name__ == '__main__':
    # Text file data converted to integer data type

    dataset_contact = []
    dataset_flight = []
    dataset_1ms = []

    list_data =["data1_fmpc_pd.txt", "data2_fmpc_pd.txt", "data3_fmpc_pd.txt", "data4_fmpc_pd.txt", "data5_fmpc_pd.txt",
                "data6_fmpc_pd.txt", "data7_fmpc_pd.txt", "data8_fmpc_pd.txt", "data9_fmpc_pd.txt", "data10_fmpc_pd.txt",
                "data15_fmpc_pd.txt", "data16_fmpc_pd.txt", "data17_fmpc_pd.txt", "data18_fmpc_pd.txt", "data19_fmpc_pd.txt",
                "data20_fmpc_pd.txt", "data21_fmpc_pd.txt", "data24_fmpc_pd.txt", "data25_fmpc_pd.txt", "data27_fmpc_pd.txt",]

    for index, dataname in enumerate(list_data):
        
        print(dataname)
        datai = np.loadtxt(dataname, dtype= float, delimiter=',')
        # print(f'state_control = {datai.shape}')

        data_contact = datai[0:800:25, :]
        for k in range(6):
            data_flight = datai[800+10*k:1100+10*k:99, :]
            dataset_flight.append(data_flight)

        # print("data_contact:", data_contact)
        dataset_contact.append(data_contact)

    for index, dataname in enumerate(list_data):
        datai = np.loadtxt(dataname, dtype=float, delimiter=',')
        dataset_1ms.append(datai)

    # Set file path to save dataset
    file_path_1 = directory + '/fmpc_pd/contact-25ms/dataset.pkl'
    save_to_pickle(np.array(dataset_contact), file_path_1)
    file_path_2 = directory + '/fmpc_pd/flight-100ms/dataset.pkl'
    save_to_pickle(np.array(dataset_flight), file_path_2)
    file_path_3 = directory + '/fmpc_pd/full-1ms/dataset.pkl'
    save_to_pickle(np.array(dataset_1ms), file_path_3)
    print('All Exported done!!')
