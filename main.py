
from typing import Dict, Any

import numpy as np
# import pandas as pd
import matplotlib.pyplot as plt
import networkx as nx
import cv2
from networkx import Graph


def toarray(fichier):
  myarray = np.loadtxt(fichier, delimiter=";", dtype=str)
  return myarray


def chop_header(myarray, a_un_entete):
  newarray = np.stack(myarray)  # recopie du array myarray
  if a_un_entete:
    print("suppression du premier element du tableau  : ", myarray[0])
    newarray = myarray[1:]  # recopie de myarray sans la premiere ligne
  
  else:
    newarray = myarray
  # print("le nouveau tableau est maintenant : ", newarray[0:1])
  return newarray


def construitgraphe(myarray, routes):
  nombreelements = len(myarray)
  # print("nombre d'éléments:", nombreelements)
  
  graph: Graph = nx.Graph()
  colors: Dict = {'a': 'red', 'd': 'green'}
  if routes == 'a':
    for i in range(nombreelements):
      if myarray[i][2] == 'a':
        graph.add_edge(myarray[i][0], myarray[i][1], type=myarray[i][2], duree=float(myarray[i][3]),
                       cout=float(myarray[i][4]), color='red')
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
      else:
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
    return graph
  elif routes == 'd':
    for i in range(nombreelements):
      if myarray[i][2] == 'd':
        graph.add_edge(myarray[i][0], myarray[i][1], type=myarray[i][2], duree=float(myarray[i][3]),
                       cout=float(myarray[i][4]), color='green')
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
      else:
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
    return graph
  else:
    for i in range(nombreelements):
      if myarray[i][2] == 'd':
        graph.add_edge(myarray[i][0], myarray[i][1], type=myarray[i][2], duree=float(myarray[i][3]),
                       cout=float(myarray[i][4]), color='green')
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
      elif myarray[i][2] == 'a':
        graph.add_edge(myarray[i][0], myarray[i][1], type=myarray[i][2], duree=float(myarray[i][3]),
                       cout=float(myarray[i][4]), color='red')
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
      else:
        graph.add_node(myarray[i][0])
        graph.add_node(myarray[i][1])
    return graph


def construitpos(myarray):
  posvilles: dict[str, Any] = {}
  nombreelements = len(myarray)
  # print("nombre d'éléments:", nombreelements)
  
  for idxville in range(nombreelements):
    # print("intégration de : ", (myarray[i][0]), (myarray[i][1]), (myarray[i][2]))
    posvilles[(myarray[idxville][0])] = (int(myarray[idxville][1]), int(myarray[idxville][2]))
  
  return posvilles


def affichage(graph, bck_img):
  
  plt.figure(str(graph))
  plt.rcParams["figure.figsize"] = (60, 15)
  plt.axis("off")
  plt.tight_layout()
  
  # on commence par afficher l'image de fond
  plt.imshow(bck_img)
  
  # utilisation d'une position bidon pour faire un premier affichage
  # pos = nx.spring_layout(graph, seed=13, k=1.666)  # positions for all nodes - seed for reproducibility
  
  nx.draw_networkx_nodes(graph, pos=posv)
  # on dessine les labels des noeuds:
  nx.draw_networkx_labels(graph, pos=posv)
  colors = []
  for edge in graph.edges:
    colors.append(graph.edges[edge]['color'])
  # colors = [graph.edges[edge]['color'] for edge in graph.nodes]
  nx.draw_networkx_edges(graph, pos=posv, width=2, edge_color=colors)
  # nx.draw_networkx_labels(graph, pos=newpos, labels=labels, verticalalignment='center')

  # nx.draw_networkx_edge_labels(g, pos=pos)

  # preparation de l'affichage secondaire ordonné:
  # ax = plt.subplots()
  # ax.margins(0.08)


def estconnexe(graph):
  if nx.is_connected(graph):
    return True
  else:
    return False

def peutatteindre(graph,villedepart,villearrivee,dejavisite):
  if villearrivee in graph.neighbors(villedepart):
    return True
  else:
    # print("reste : ", (set(graph.neighbors(villedepart))) - set(dejavisite))
    for node in graph.neighbors(villedepart):
      if node not in dejavisite:
        dejavisite.append(node)
        # print(str(node), "visité ! ")
        if peutatteindre(graph,node,villearrivee,dejavisite):
          return True
    return False
  
  
def estconnexe(graph,villedepart):
  connexe=True
  for node in graph.nodes:
    if node != villedepart:
      if peutatteindre(graph,villedepart,node,[node]):
        connexe=connexe&True
      else:
        return False
  return connexe


'''
def cheminpluscourt(graph,villedepart,villearrivee)
  chemin = {}
  cout = 0
  dictcouts={villedepart:0}
  def disjkstra(graph,villedepart,dictcout):
    for node in sorted(graph.neighbors(villedepart), :
      if node not in dictcout :
        if dictcout{villedepart[0]} > graph.edges[villedepart,node]['weight']+dictcout
          dictcout.append(node:{graph.edges[villedepart,node]['weight']+dictcout(villedepart),villedepart}
        
  
  
  return chemin,cout
'''

# TODO : pseudo code

## Version avec évlautation sur le noeud courant
# sur le noeud N, on évalue le cout d'arrivée, si le cout est inférieur au cout connu, on enregistre d'ou on vient, et le nouveau cout

# Version avec évaluation des voisins
# sur le noeud N, on évlue le cout d'arrivée
# on réference tous les voisins, si le cout d'arrivée jusqu'au voisin est inférieur au cout connu, on l'enregirtre, le nouveau prédécedesceseur etant le noeud courant, le cout etant le cout au neoud courant plus le cout du lien

'''
besoins :
liste des voisins
cout du lien noeud courant - voisin
cout au noeud courant
dictionnaire des couts min par noeud et des antécédents correspondants
'''


def cheminlepluscourt(graph, villedepart, villearrivee):
  chemin = nx.shortest_path(graph, source=villedepart, target=villearrivee, weight='duree')
  
  return chemin
  
  

################ main ##################
if __name__ == '__main__':
  # read string from user
  
  image = cv2.imread('./carte.jpg')
  arraypositions = toarray('./TP2_position.csv')
  arrayedges = toarray('./TP2_liaison.csv')
  
  r_entete = 'Yes'
  # r_entete = input('Les fichiers de données possedent t ils un entete (y/n) (Y)? : ' )
  if r_entete in ['n', 'N', 'no', 'NO', 'No', 'non', 'Non', 'NON']:
    a_un_entete = False
  else:
    a_un_entete = True
  
  # print(map_data.head())
  # print(graph_data.head())
  chopped_map_data = chop_header(arraypositions, a_un_entete)
  chopped_graph_data = chop_header(arrayedges, a_un_entete)
  
  # print("liste des positions : ", chopped_map_data[0:2])
  # print("mon array est maintenant : ", chopped_graph_data[0:2])
  
  posv = construitpos(chopped_map_data)
  
  # print("positions de Lille : ", posv['Lille'])
  
  G = construitgraphe(chopped_graph_data, 'tout')
  Ga = construitgraphe(chopped_graph_data, 'a')
  Gd = construitgraphe(chopped_graph_data, 'd')
  
  # print("liste des noeuds de G : ", dict(G.nodes.items()))
  # print("liste des routes du graph G : ", dict(G.edges.items()))
  
  # print("parcours de Lille à Marseille par toutes les routes : ",peutatteindre(G,'Lille','Marseille',['Lille']))
  #  print("parcours de Brest à Paris par les d : ", peutatteindre(Gd, 'Brest', 'Paris',['Brest']))
  # print("verification de la connectivité de G: ", estconnexe(G,'Lille'))
  # print("verification de la connectivité de Gd: ", estconnexe(Ga,'Lyon'))
  
  # print("le graphe G est connexe : ", estconnexe(G))
  # print("le graphe Gd est connexe : ", estconnexe(Gd))
  # print("le graphe Ga est connexe : ", estconnexe(Ga))
  
  # print(cheminlepluscourt(G, 'Nantes', 'Bordeaux'))
  print(cheminlepluscourt(G, 'Lille', 'Marseille'))

  
  listecouts:Dict={}
  listecouts['Lille']=['0','Lille']
  listecouts['Dunkerke']=['57','Lille']
  print(listecouts['Dunkerke'][0])
  
  
  
  
  depart='Lille'
  for node in sorted(Ga.neighbors(depart) ):
    print("liaison : ",depart,'-',node,", cout associé :",G.edges[depart,node]['duree'])
    print(type(G.edges[depart,node]['duree']))
    listecouts[node]=[G.edges[depart,node]['duree'],depart]
    listecouts['Lille'][1]='Bethune'
  print(listecouts)
  
  # affichage(G, image)
  # affichage(Ga, image)
  affichage(Gd, image)
  
  # plt.show()
