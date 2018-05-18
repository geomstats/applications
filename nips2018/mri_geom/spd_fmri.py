""" This file provides is an example of the application of geomstats
on the space of SDP matrices. The goal here is to classify a set of 86
brain connectomes represented by their SPD regularized Laplacian
(with parameter GAMMA) into two classes: control vs people suffering
from schizophrenia.
"""
import geomstats.spd_matrices_space as spd_space
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.cluster.hierarchy as hc
import seaborn as sb
from sklearn.metrics import accuracy_score, confusion_matrix
from sklearn.metrics import f1_score, precision_score, recall_score
from sklearn.model_selection import KFold
from sklearn.svm import SVC

import time

N_NODES = 28
RAND_SEED = 2018
SPACE = spd_space.SPDMatricesSpace(n=N_NODES)
CORR_THRESH = 0.1
GAMMA = 1.0
N_GRAPHS = 86
SIGMAS = list(np.arange(0.5, 20, 0.5))
DISTANCES = ['log_euclidean', 'frobenius', 'riemannian']
TRAIN_SIZE = 0.85


def import_data():
    graphs = pd.DataFrame.from_csv('data/train_fnc.csv')
    map_functional = pd.DataFrame.from_csv('add_info/comp_ind_fmri.csv',
                                           index_col=None)
    map_functional = map_functional['fMRI_comp_ind'].to_dict()
    map_functional_r = {v: k for k, v
                        in map_functional.items()}
    mapping = pd.DataFrame.from_csv('add_info/' +
                                    'rs_fmri_fnc_mapping.csv')
    graph_labels = pd.DataFrame.from_csv('data/train_labels.csv')
    all_graphs = [None] * N_GRAPHS
    all_targets = np.zeros(N_GRAPHS)

    def create_connectome(graph_id, mapping):
        u = np.zeros((DIM_SPACE, DIM_SPACE))
        nb_edges = mapping.shape[0]
        for edge in range(nb_edges):
            e0, e1 = (mapping.iloc[edge]['mapA'], mapping.iloc[edge]['mapB'])
            region0, region1 = map_functional_r[e0], map_functional_r[e1]
            u[region0, region1] = graphs.iloc[graph_id][edge]
            u = np.multiply(u, (u > CORR_THRESH))
        return np.abs(u + u.T)

    for graph_id in range(N_GRAPHS):
        all_graphs[graph_id] = create_connectome(graph_id, mapping)
        all_targets[graph_id] = int(graph_labels.loc[graphs.index[graph_id], 'Class'])

    all_targets = np.array(all_targets)

    np.random.seed(RAND_SEED)
    index_train = list(range(NB_GRAPHS))
    np.random.shuffle(index_train)
    stop = int(TRAIN_SIZE * N_GRAPHS)
    labels = all_targets[index_train]
    return (graphs.iloc[:, index_train], [all_graphs[t] for t in index_train],
            labels, stop, index_train)


def laplacian(a):
    d = np.diag(np.array(a.sum(1)).flatten())
    return d-a


def frobenius(hat_l1, hat_l2):
    return np.linalg.norm(spd_space.group_log(hat_l1).squeeze(0)
                          - spd_space.group_log(hat_l2).squeeze(0),
                          'fro')


def riemannian(inv_l1, hat_l2):
    return np.linalg.norm(inv_l1.dot(spd_space.group_log(hat_l2).squeeze(0).dot(inv_l1)),
                          'fro')


def log_euclidean(hat_l1, hat_l2):
    return np.linalg.norm(spd_space.group_log(hat_l1).squeeze(0)
                          - spd_space.group_log(hat_l2).squeeze(0),
                          'fro')


def fit_kernel_cv(log_euclidean_distance, labels,
                  sigma=None, verbose=False):
    kf = KFold(n_splits=10)
    perf = {}
    conf = {}
    nb_train = len(labels)
    if sigma is not None:
        distance = np.exp(- np.square(log_euclidean_distance)/(sigma**2))
    else:
        distance = log_euclidean_distance

    k = 0
    for train_index, test_index in kf.split(range(nb_train)):
        train_index = np.array(train_index)
        test_index = np.array(test_index)
        x = np.array(distance)[train_index, :][:, train_index]
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


def compute_similarities(all_graphs, type_dist):
    distance = np.zeros((N_GRAPHS, N_GRAPHS))
    for c, g in enumerate(all_graphs):
            hat_l1 = laplacian(g) + GAMMA * np.eye(DIM_SPACE)
            lambda2, u = np.linalg.eigh(hat_l1)
            inv_l1 = u.dot(np.diag(1.0/np.sqrt(lambda2)).dot(u.T))
            for j in range(c):
                hat_l2 = laplacian(all_graphs[j]) + GAMMA * np.eye(DIM_SPACE)
                if type_dist == 'log_euclidean':
                    distance[c, j] = log_euclidean(hat_l1, hat_l2)
                elif type_dist == 'frobenius':
                    distance[c, j] = frobenius(hat_l1, hat_l2)
                else:
                    distance[c, j] = riemannian(inv_l1, hat_l2)
    return distance + distance.T


def classify_graphs():
    time_alg = {}
    graphs, all_graphs, labels, stop, index_train = import_data()
    distance = {type_dist: np.zeros((N_GRAPHS, N_GRAPHS))
                for type_dist in DISTANCES}

    tic = time.time()
    for type_dist in DISTANCES:
        tic = time.time()
        distance[type_dist] = compute_similarities(all_graphs, type_dist)
        toc = time.time()
        time_alg[type_dist] = toc-tic
    print('Done computing distances')

    print('Starting SVM fitting with Cross Validation')
    mean_acc = {}
    for type_dist in DISTANCES:
        mean_acc[type_dist] = []
        for sigma in SIGMAS:
            perf, conf = fit_kernel_cv(distance[type_dist][:stop, :stop], labels[:stop],
                                       sigma, verbose=False)
            mean_acc[type_dist].append(np.mean([perf[k]['acc']
                                                for k in perf.keys()]))

    print('Fitting SVM on full data and evaluation on OOS set.')
    model = {}
    perf_final = {}
    sigma_chosen = {}
    for type_dist in DISTANCES:
        sigma_chosen[type_dist] = SIGMAS[np.argmax(mean_acc[type_dist])]
        distance2 = np.exp(- np.square(distance[type_dist])
                           / (sigma_chosen[type_dist]**2))
        x = np.array(distance2[:stop, :])[:, :stop]
        x_test = np.array(distance2[stop:, :])[:, :stop]
        clf = SVC(kernel='precomputed')
        clf.fit(x, labels[:stop])
        model[type_dist] = clf
        y_test = clf.predict(x_test)
        perf_final[type_dist] = {'acc': accuracy_score(labels[stop:], y_test),
                                 'prec': precision_score(labels[stop:], y_test),
                                 'f1': f1_score(labels[stop:], y_test),
                                 'recall': recall_score(labels[stop:], y_test)}

    return distance, time_alg, perf_final, sigma_chosen, mean_acc, labels


if __name__ == '__main__':

    distance_dict, _, perf_final, sigma_chosen, mean_acc, labels = classify_graphs()
    print('Final performance of the model on OOS test data:')
    print(pd.DataFrame.from_dict(perf_final))

    # Plot CV accuracy
    fig, ax = plt.subplots(figsize=(6, 4))
    for type_dist in DISTANCES:
        plt.plot(SIGMAS, mean_acc[type_dist],
                 label=type_dist + ' \nDistance')
    plt.legend(loc='center left', bbox_to_anchor=(1.0, 0.5), fontsize=18)
    plt.xlabel(r'$\sigma$', fontsize=24)
    plt.ylabel(r'CV accuracy', fontsize=20)
    plt.show()

    # Plot Clustermap representation
    for type_dist in DISTANCES:
        plt.figure()
        kernel_dist = np.exp(-np.square(distance_dict[type_dist])
                             / (sigma_chosen[type_dist]**2))
        linkage = hc.linkage(kernel_dist, method='average')
        sb.clustermap(kernel_dist, row_linkage=linkage, col_linkage=linkage,
                      cmap='coolwarm', xticklabels=labels, yticklabels=labels)
        plt.title('Clustermap for the ' + type_distance + ' distance')
        plt.show()
