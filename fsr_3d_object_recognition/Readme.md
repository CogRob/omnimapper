## 3D Object Recognition Using RANSAC

This implements the object recognition in papers:

C. Papazov and D. Burschka "An Efficient RANSAC for 3D Object Recognition in Noisy and Occluded Scenes"

C. Papazov, S. Haddadin, S. Parusel ,K. Krieger, D. Burschka "Rigid 3D geometry matching for grasping of known objects in cluttered scenes."

It uses surface features that are saved in a hash table to match surfaces of objects in a scene to models that are stored in a database. It also estimates the pose of the object in the scene.

Two algorithms are used. The first is an offline algorithm that computes geometric descriptors for objects that are to be recognized in the online phase. The descriptors are saved in a hash table for fast lookup in the online phase. The second algorithm is the online recognition algorithm. It samples point pairs from the scene, computes the descriptor between them, and uses that descriptor to lookup the model in the hash table if it's a good match. It does this several times and saves the models as hypotheses which are verified in the last stage of the algorithm.


### Offline Algorithm (Feature Description)

__Input :__ a model $M$
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; a hash table $H$ of point pairs keyed by the feature descriptor
__Output :__ modified $H$

1. __for each__ point $p_{u} \in$ $M$
2. &nbsp;&nbsp;&nbsp;&nbsp;__for each__ point $p_{v} \in$ $M$ at a distance [$d-\delta$, $d+\delta$] from $p_{u}$
3. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Estimate normals $n_{u}$ and $n_{v}$ of $p_{u}$ and $p_{v}$
4. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Set $opp=((p_{u},n_{u}),(p_{v},n_{v}))$
5. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Compute feature:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp;&nbsp;$f(opp)=\left(\begin{array}{ccc}
||p_{u}-p_{v}|| \\
\angle(n_{u},n_{v}) \\
\angle(n_{u},p_{v}-p_{u}) \\
\angle(n_{v},p_{u}-p_{v}) \\
\end{array}\right)$
6. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Compute the     local coordinate system $F$ of $opp$
7. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Add key value pair $(f, (opp, F_{u,v}))$ to $H$
8. __return__ $H$

### Online Algorithm (Object Recognition)

__Input :__ a set of models $O = \{M_{1}, M_{2}, \ldots\}$
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;a scene $S$
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;hash table $H$ from the offline phase
__Output :__ a set of models-transform pairs $R=\{(M_{1},T_{1}),(M_{2},T_{2}),\ldots\}$

1. Compute octree $S_{O}$ of $S$ to reduce the scene
2. Compute the number of iterations $N$
3. __do__ $N$ times
4. &nbsp;&nbsp;&nbsp;&nbsp;Sample point $p_{u}$ from $S_{O}$
5. &nbsp;&nbsp;&nbsp;&nbsp;Compute set of points $L=\{x\in$  $S_{O}~~|~~||x-p_{u}|| \in$ $[d-\delta,d+\delta]\}$
6. &nbsp;&nbsp;&nbsp;&nbsp;Sample point $p_{v}$ from $L$
7. &nbsp;&nbsp;&nbsp;&nbsp;Estimate normals $n_{u}$ and $n_{v}$ of $p_{u}$ and $p_{v}$
8. &nbsp;&nbsp;&nbsp;&nbsp;Set $opp=((p_{u},n_{u}),(p_{v},n_{v}))$
9. &nbsp;&nbsp;&nbsp;&nbsp;Compute feature $f(opp)$ and local coordinate system $F$ for $opp$
10. &nbsp;&nbsp;&nbsp;&nbsp;__for each__ $((u_{i},v_{i}),F_{i}) \in$ $H(f)$
11. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Get model $M$ of $(u_{i},v_{j})$
12. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Compute $T$ that aligns $(u_{i},v_{i})$ to $opp$
13. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;If $accept(M,T)$
14. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$R\leftarrow R\cup(M,T)$
15. Remove confliting hypotheses from $R$
16. __return__ $R$
