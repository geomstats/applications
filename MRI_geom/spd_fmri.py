import geomstats.spd_matrices_space as spd_space
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.metrics import accuracy_score, confusion_matrix
from sklearn.metrics import f1_score, precision_score, recall_score 
from sklearn.model_selection import KFold
from sklearn.svm import SVC
import time

DIM_SPACE = 28
RAND_SEED = 2018
SPACE = spd_space.SPDMatricesSpace(dimension=DIM_SPACE)
CORR_THRESH = 0.1
GAMMA = 1.0
NB_GRAPHS = 86
SIGMAS = list(np.arange(0.5, 20, 0.5))
DISTANCE = ["log_euclidean", "frobenius", "riemannian"]

def import_data():
    graphs = pd.DataFrame.from_csv("data/train_fnc.csv")
    map_FCN = pd.DataFrame.from_csv("add_info/comp_ind_fmri.csv",
                                    index_col=None)
    map_FCN = map_FCN['fMRI_comp_ind'].to_dict()
    map_FCN_r = {v: k for k, v
                 in map_FCN.iteritems()}
    mapping = pd.DataFrame.from_csv("add_info/" +
                                    "rs_fmri_fnc_mapping.csv")
    graph_labels = pd.DataFrame.from_csv("data/train_labels.csv")
    all_graphs = [None] * NB_GRAPHS
    all_targets = [None] * NB_GRAPHS
    
    def create_connectome(graph_id, mapping):
        u = np.zeros((DIM_SPACE, DIM_SPACE))
        nb_edges = mapping.shape[0]
        for edge in range(nb_edges):
            e0, e1 = (mapping.iloc[edge]["mapA"], mapping.iloc[edge]["mapB"])
            u[map_FCN_r[e0], map_FCN_r[e1]] = graphs.iloc[graph_id][edge]
            u = np.multiply(u, (u > CORR_THRESH))
        return np.abs(u + u.T)
                       
        
    for graph_id in range(NB_GRAPHS):
        all_graphs[graph_id] = create_connectome(graph_id, mapping)
        all_targets[graph_id] = graph_labels.loc[graphs.index[graph_id], "Class"]

    np.random.seed(RAND_SEED)
    index_train = range(NB_GRAPHS)
    np.random.shuffle(index_train)
    stop = int(0.85 * NB_GRAPHS)
    labels = np.array(all_targets[index_train])
    return (graphs.iloc[:, index_train], [all_graphs[t] for t in index_train],
        labels, stop, index_train)


def laplacian(a):
    d = np.diag(np.array(a.sum(1)).flatten())
    return d-a


def fit_kernel_cv(log_euclidean_distance, labels,
                  sigma=None, verbose=False):
    kf = KFold(n_splits=10)
    perf = {}
    conf = {}
    k = 0
    if sigma is not None:
        distance = np.exp(- np.square(log_euclidean_distance)/(sigma**2))
    else:
        distance = log_euclidean_distance
    for train_index, test_index in kf.split(range(NB_GRAPHS)):
        train_index = np.array(train_index)
        test_index = np.array(test_index)
        x = np.array(distance[train_index, :])[:, train_index]
        x_test = np.array(distance[test_index, :])[:, train_index]
        clf = SVC(kernel='precomputed')
        clf.fit(x, labels[train_index])
        y_train = clf.predict(x)
        if verbose:
            print("Training accuracy:",
                  accuracy_score(labels[train_index], y_train))
        y_test = clf.predict(x_test)
        perf[k] = {'acc': accuracy_score(labels[test_index], y_test),
                   'prec': precision_score(labels[test_index], y_test),
                   'f1': f1_score(labels[test_index], y_test),
                   'recall': recall_score(labels[test_index], y_test)}
        conf[k] = confusion_matrix(labels[test_index], y_test)
        k += 1
        return perf, conf


def main():
    gamma = GAMMA
    time_alg = {}
    graphs, all_graphs, labels, stop, index_train = import_data()
    distance = {option: np.zeros((NB_GRAPHS, NB_GRAPHS))
                for option in DISTANCE}
    tic = time.time()
    for option in DISTANCE:
        tic = time.time()
        for c, g in enumerate(all_graphs):
            hat_l1 = laplacian(g) + gamma*np.eye(DIM_SPACE)
            lambda2, u = np.linalg.eigh(hat_l1)
            inv_l1 = u.dot(np.diag(1.0/np.sqrt(lambda2)).dot(u.T))
            for j in range(c):
                hat_l2 = laplacian(all_graphs[j]) + gamma*np.eye(DIM_SPACE)
                if option == "log_euclidean":
                    distance[option][c, j] = np.linalg.norm(spd_space.group_log(hat_l1).squeeze(0)
                                                            - spd_space.group_log(hat_l2).squeeze(0),
                                                            'fro')
                elif option == "frobenius":
                    distance[option][c, j] = np.linalg.norm(hat_l1
                                                            - hat_l2, 'fro')
                else:
                    distance[option][c, j] = np.linalg.norm(
                                                            inv_l1.dot(spd_space.group_log(hat_l2).squeeze(0).dot(inv_l1)),
                                                            'fro')
        distance[option] = distance[option] + distance[option].T
        toc = time.time()
        time_alg[option] = toc-tic
    print("Done computing distances")

    print("Starting SVM fitting with Cross Validation")
    sigmas = SIGMAS
    mean_acc = {}
    for option in DISTANCE:
        print("Option: ", option)
        mean_acc[option] = []
        perf, conf = fit_kernel_cv(distance[option][:stop, :stop],
                                   labels[:stop], sigma=None, verbose=False)
        mean_acc[option].append(np.mean([perf[k]['acc'] for k in perf.keys()]))
        for sigma in sigmas:
            perf, conf = fit_kernel_cv(distance[option][:stop, :stop],
                                       labels[:stop], sigma, verbose=False)
            mean_acc[option].append(np.mean([perf[k]['acc']
                                             for k in perf.keys()]))

    print("Fitting SVM on full data and evaluation on OOS set.")
    model = {}
    perf_final = {}
    sigma_chosen = {}
    for option in DISTANCE:
        sigma_chosen[option] = sigmas[np.argmax(mean_acc[option])] - 1
        distance2 = np.exp(- np.square(distance[option])
                           / (sigma_chosen[option]**2))
        x = np.array(distance2[:stop, :])[:, :stop]
        x_test = np.array(distance2[stop:, :])[:, :stop]
        clf = SVC(kernel='precomputed')
        clf.fit(x, labels[:stop])
        model[option] = clf
        y_test = clf.predict(x_test)
        print(y_test)
        perf_final[option] = {'acc': accuracy_score(labels[stop:], y_test),
                              'prec': precision_score(labels[stop:], y_test),
                              'f1': f1_score(labels[stop:], y_test),
                              'recall': recall_score(labels[stop:], y_test)}

    return distance, time_alg, perf_final, sigma_chosen, mean_acc, labels


if __name__ == "__main__":

    distance, _, perf_final, sigma_chosen, mean_acc, labels = main()
    print("Final performance of the model on OOS test data:")
    print(pd.DataFrame.from_dict(perf_final))

    # Plot CV accuracy
    fig, ax = plt.subplots(figsize=(6, 4))
    for option in DISTANCE:
        plt.plot(SIGMAS, mean_acc['log_euclidean'][1:],
                 label=option + " \nDistance")
    plt.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), fontsize=18)
    plt.xlabel(r'$\sigma$', fontsize=24)
    plt.ylabel(r'CV accuracy', fontsize=20)

    # Plot Clustermap representation
    import seaborn as sb
    import scipy.cluster.hierarchy as hc
    for option in DISTANCE:
        plt.figure()
        distance2 = np.exp(-np.square(distance[option])
                           / (sigma_chosen[option]**2))
        linkage = hc.linkage(distance2, method='average')
        sb.clustermap(distance2, row_linkage=linkage, col_linkage=linkage,
                      cmap="coolwarm", xticklabels=labels, yticklabels=labels)
