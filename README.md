# Interactive ARAP

***As-rigid-as-possible surface deformation* (ARAP)** is *an iterative mesh processing scheme* in which *the shape is stretched or sheared* while *the small parts of the object are preserved locally*, i.e., small parts are modified to be ***as rigid as possible***. In our work, we implement this **ARAP** deformation algorithm by referring to the work of Sorkine et al [[_1_]](https://igl.ethz.ch/projects/ARAP/arap_web.pdf). Our motivation for choosing this topic was that it is interactive and independent of additional hardware. In the [***report***](https://github.com/erdenbatuhan/interactive-arap/blob/master/doc/report.pdf), we review related work, the methods used for the algorithm, our results, and the conclusion, which includes the challenges we encountered, and our future work.

### Results

**// TODO: Gif here**

_You can find more pictures of the results [here](https://github.com/erdenbatuhan/interactive-arap/blob/master/doc/img)_!

### Local Environment Setup

You can easily download and run the project without installing a lot of libraries. Please use the following instructions to do so.

##### OpenGL

You need to have **OpenGL** installed on your system to build and run the project.

##### libigl

The library, **[libigl](https://libigl.github.io/tutorial/)**, will be installed by itself (*@see* the corresponding cmake module: **./cmake/libigl**).

##### OpenMP (Optional)

If you have **OpenMP** installed on your system, the project will run in parallel.

### Contributors

- Erden, Batuhan
- Epple, Alexander
- Shahzad, Anas
- Yildirim, Cansu

### References

[[_1_]](https://igl.ethz.ch/projects/ARAP/arap_web.pdf) Olga Sorkine and Marc Alexa. As-rigid-as-possible surface modeling. In *Proceedings of the Fifth Eurographics Sym- posium on Geometry Processing*, SGP ’07, page 109–116, Goslar, DEU, 2007. Eurographics Association. 1

[[_2_]](http://www.cad.zju.edu.cn/home/bao/pub/Cage-based_deformation_transfer.pdf) LuChen, JinHuang, HanqiuSun, and HujunBao. Cage-based deformation transfer. *Comput. Graph.*, 34:107–118, 2010. 1

[[_3_]](https://people.inf.ethz.ch/~sumnerb/research/embdef/Sumner2007EDF.pdf) Robert W. Sumner, Johannes Schmid, and Mark Pauly. Embedded deformation for shape manipulation. *ACM Trans.* *Graph.*, 26(3):80–es, jul 2007. 1

[[_4_]](https://www.cs.jhu.edu/~misha/Fall07/Papers/Yu04.pdf) Yizhou Yu, Kun Zhou, Dong Xu, Xiaohan Shi, Hujun Bao, Baining Guo, and Heung-Yeung Shum. Mesh editing with poisson-based gradient field manipulation.  *ACM Trans. Graph.*, 23(3):644–651, aug 2004. 1
