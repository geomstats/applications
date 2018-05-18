## Geomstats for Graph Classification ##

This example shows how to leverage the Riemannian geometry for a brain connectome  classification challenge. The dataset consists in the resting-state fMRIs of 86 patients  at 28 Regions Of Interest (ROIs), and the goal is to compare  and classify the induced graph regularized laplacians (a special case of SPD matrices).

To run the example from the geomstat directory, enter in the terminal:

```
cd examples/nips2018/mri_geom
python spd_fmri.py
```


Dependencies:
+ pandas 
+ seaborn
