import re
from typing import Dict, Any

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import networkx as nx
import cv2
from networkx import Graph

bck_img = cv2.imread('./carte.jpg')

map_data = pd.read_csv('./TP2_position.csv')
graph_data = pd.read_csv('./TP2_liaison.csv')

print(map_data.head())
print(graph_data.head())


def toarray(fichier):
    myarray = np.loadtxt(fichier, delimiter=";", dtype=str)
    return myarray


arraypositions = toarray('./TP2_position.csv')
arrayedges = toarray('./TP2_liaison.csv')


def chop_header(myarray, a_un_entete):
    newarray = np.stack(myarray)  # recopie du array myarray
    if a_un_entete:
        print("suppression du premier element du tableau  : ", myarray[0])
        newarray=myarray[1:] # recopie de myarray sans la premiere ligne

    else:
        newarray = myarray
    print("le nouveau tableau est maintenant : ", newarray[0:1])
    return newarray


chopped_map_data = chop_header(arraypositions, True)
chopped_graph_data = chop_header(arrayedges, True)

print("liste des positions : ", chopped_map_data[0:2])
print("mon array est maintenant : ", chopped_graph_data[0:2])


def construitgraphe(myarray):
    nombreelements = len(myarray)
    # print("nombre d'éléments:", nombreelements)

    graph: Graph = nx.Graph()

    for i in range(nombreelements):
        # print("intégration de : ", (myarray[i][0]), (myarray[i][1]), (myarray[i][2]))
        graph.add_edge(myarray[i][0], myarray[i][1], type=myarray[i][2], duree=float(myarray[i][3]),
                       cout=float(myarray[i][4]))
    return graph


def construitpos(myarray):
    posvilles: dict[str, Any] = {}
    nombreelements = len(myarray)
    print("nombre d'éléments:", nombreelements)

    for idxville in range(nombreelements):
        # print("intégration de : ", (myarray[i][0]), (myarray[i][1]), (myarray[i][2]))
        posvilles[(myarray[idxville][0])] = (int(myarray[idxville][1]), int(myarray[idxville][2]))

    return posvilles

posv = construitpos(chopped_map_data)

print("positions de Lille : ", posv['Lille'])

G = construitgraphe(chopped_graph_data)

print("liste des noeuds de G : ", dict(G.nodes.items()))
print("contenu du graph G : ", dict(G.edges.items()))

# on commence par afficher l'image de fond
plt.imshow(bck_img)

# utilisation d'une position bidon pour faire un premier affichage
pos = nx.spring_layout(G, seed=13, k=1.666)  # positions for all nodes - seed for reproducibility

nx.draw_networkx_nodes(G, pos=posv)
nx.draw_networkx_edges(G, pos=posv, width=2)
# on dessine les labels :
# nx.draw_networkx_labels(G, pos=pos, horizontalalignment='center')

# modification de la position entre deux affichages :

# nx.draw_networkx_labels(G, pos=newpos, labels=labels, verticalalignment='center')

# nx.draw_networkx_edge_labels(g, pos=pos)


# preparation de l'affichage secondaire ordonné:
# ax = plt.subplots()
# ax.margins(0.08)
plt.axis("on")

# plt.tight_layout()
plt.show()
