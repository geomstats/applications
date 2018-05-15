import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time
from sklearn.svm import SVC
from sklearn.model_selection import KFold
from sklearn.metrics import precision_score, accuracy_score
from sklearn.metrics import f1_score, recall_score, confusion_matrix
import geomstats.spd_matrices_space as spd_space
SPACE = spd_space.SPDMatricesSpace(dimension=28)


graphs = pd.DataFrame.from_csv("data/train_FNC.csv")
map_FCN = pd.DataFrame.from_csv("AdditionalInformation/comp_ind_fMRI.csv",
                                index_col=None)
map_FCN = map_FCN['fMRI_comp_ind'].to_dict()
map_FCN_r = {v: k for k, v in map_FCN.iteritems()}
mapping = pd.DataFrame.from_csv("AdditionalInformation/rs_fMRI_FNC_mapping.csv")
graph_labels = pd.DataFrame.from_csv(
                                     "data/train_labels.csv")
all_graphs = [None]*graphs.shape[0]
all_targets = [None]*graphs.shape[0]
vpos = np.vectorize(lambda x: max(0, x))
for i in range(graphs.shape[0]):
    u = np.zeros((28, 28))
    for ii in range(mapping.shape[0]):
        a, b = (mapping.iloc[ii]["mapA"], mapping.iloc[ii]["mapB"])
        u[map_FCN_r[a], map_FCN_r[b]] = graphs.iloc[i][ii]
    u = np.multiply(u, (u > 0.0))
    all_graphs[i] = nx.from_numpy_array(u + u.T)
    all_targets[i] = graph_labels.loc[graphs.index[i], "Class"]


np.random.seed(2018)
index_train = range(len(all_graphs))
np.random.shuffle(index_train)
T = int(0.85*len(all_graphs))
labels = np.array([all_targets[t] for t in index_train])


def Laplacian(A):
    D = np.diag(np.array(A.sum(1)).flatten())
    return D-A


def main():
    gamma = 1
    time_alg = {}
    distance = {option: np.zeros((len(all_graphs), len(all_graphs)))
                for option in ["LED", "ED", "RD"]}
    tic = time.time()
    for option in ["LED", "ED", "RD"]:
        tic = time.time()
        for i in range(1, len(all_graphs)):
            hat_l1 = Laplacian(nx.adjacency_matrix(all_graphs[index_train[i]]).todense())\
                     + gamma*np.eye(28)
            Lambd, U = np.linalg.eigh(hat_l1)
            invL1 = U.dot(np.diag(1.0/np.sqrt(Lambd)).dot(U.T))
            for j in range(i):
                hat_l2 = Laplacian(nx.adjacency_matrix(
                                   all_graphs[index_train[j]]).todense())\
                        + gamma*np.eye(28)
                if option == "LED":
                    distance[option][i, j] = np.linalg.norm(spd_space.group_log(hat_l1).squeeze(0)
                                                            - spd_space.group_log(hat_l2).squeeze(0),
                                                            'fro')
                elif option == "ED":
                    distance[option][i, j] = np.linalg.norm(hat_l1
                                                            - hat_l2, 'fro')
                else:
                    distance[option][i, j] = np.linalg.norm(
                                             invL1.dot(spd_space.group_log(hat_l2).squeeze(0).dot(invL1)),
                                             'fro')
        distance[option] = distance[option] + distance[option].T
        toc = time.time()
        time_alg[option] = toc-tic

    print("Done computing distances")

    def fit_kernel_cv(log_euclidean_distance, labels,
                      sigma=None, verbose=False):
        kf = KFold(n_splits=10)
        perf = {}
        conf = {}
        k = 0
        sq = np.vectorize(lambda x: x**2)
        if sigma is not None:
            distance = np.exp(-sq(log_euclidean_distance)/(sigma**2))
        else:
            distance = log_euclidean_distance
        for train_index, test_index in kf.split(range(len(labels))):
            train_index = np.array(train_index)
            test_index = np.array(test_index)
            X = np.array(distance[train_index, :])[:, train_index]
            X_test = np.array(distance[test_index, :])[:, train_index]
            clf = SVC(kernel='precomputed')
            clf.fit(X, labels[train_index])
            y_train = clf.predict(X)
            if verbose:
                print("Training accuracy:",
                      accuracy_score(labels[train_index], y_train))
            y_test = clf.predict(X_test)
            perf[k] = {'acc': accuracy_score(labels[test_index], y_test),
                       'prec': precision_score(labels[test_index], y_test),
                       'f1': f1_score(labels[test_index], y_test),
                       'recall': recall_score(labels[test_index], y_test)}
            conf[k] = confusion_matrix(labels[test_index], y_test)
            k += 1
        return perf, conf

    print("Starting SVM fitting.")
    sigmas = list(np.arange(0.5, 10, 0.5))
    sigmas += [15, 20, 25, 30, 35, 40, 45, 50, 60, 80, 100, 120]
    mean_acc = {}
    for option in ["LED", "ED", "RD"]:
        print("Option: ", option)
        mean_acc[option] = []
        perf, conf = fit_kernel_cv(distance[option][:T, :T],
                                   labels[:T], sigma=None, verbose=False)
        mean_acc[option].append(np.mean([perf[k]['acc'] for k in perf.keys()]))
        for sigma in sigmas:
                print(sigma)
                perf, conf = fit_kernel_cv(distance[option][:T, :T],
                                           labels[:T], sigma, verbose=False)
                mean_acc[option].append(np.mean([perf[k]['acc']
                                        for k in perf.keys()]))

    return distance, mean_acc


if __name__ == "__main__":

    distance, mean_acc = main()
    sigmas = list(np.arange(0.5, 10, 0.5)) + \
        [15, 20, 25, 30, 35, 40, 45, 50, 60, 80, 100, 120]
    # Plot CV accuracy
    fig, ax = plt.subplots(figsize=(6, 4))
    plt.plot(sigmas, mean_acc['LED'][1:],
             label="Log-Euclidean \nDistance", c="blue")
    plt.plot(sigmas, mean_acc['ED'][1:],
             label="Euclidean \nDistance", c="black")
    plt.plot(sigmas, mean_acc['RD'][1:],
             label="Riemannian \nDistance", c="red")
    plt.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), fontsize=18)
    plt.xlabel(r'$\sigma$', fontsize=24)
    plt.ylabel(r'CV accuracy', fontsize=20)
    import seaborn as sb
    import scipy.cluster.hierarchy as hc
    for option in ['LED', 'ED', 'RD']:
        sigma = sigmas[np.argmax(mean_acc[option])]
        sq = np.vectorize(lambda x: x**2)
        plt.figure()
        distance2 = np.exp(-sq(distance[option])/(sigma**2))
        linkage = hc.linkage(distance2, method='average')
        sb.clustermap(distance2, row_linkage=linkage, col_linkage=linkage,
                      cmap="coolwarm", xticklabels=labels, yticklabels=labels)
 
