## 3D Object Recognition Using RANSAC

This implements the object recognition in papers:

C. Papazov and D. Burschka "An Efficient RANSAC for 3D Object Recognition in Noisy and Occluded Scenes"

C. Papazov, S. Haddadin, S. Parusel ,K. Krieger, D. Burschka "Rigid 3D geometry matching for grasping of known objects in cluttered scenes."

It uses surface features that are saved in a hash table to match surfaces of objects in a scene to models that are stored in a database. It also estimates the pose of the object in the scene.

Two algorithms are used. The first is an offline algorithm that computes geometric descriptors for objects that are to be recognized in the online phase. The descriptors are saved in a hash table for fast lookup in the online phase. The second algorithm is the online recognition algorithm. It samples point pairs from the scene, computes the descriptor between them, and uses that descriptor to lookup the model in the hash table if it's a good match. It does this several times and saves the models as hypotheses which are verified in the last stage of the algorithm.
