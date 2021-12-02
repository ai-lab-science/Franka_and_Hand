import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat


def plot_gaussian_ellipse(mean,cov):
    """
    Plots the gaussian ellipsoid for the provided covariance, centred at the provided mean. Based on the input data, the
    function is able to plot both, 2D and 3D gaussians. For the 3D Gaussian, the wireframe of the ellipsoid is plotted.
    The code for 2D Gaussian is taken from:
    https://scipython.com/book/chapter-7-matplotlib/examples/bmi-data-with-confidence-ellipses/
    The code for 3D Gaussian is taken from:
    https://stackoverflow.com/questions/41955492/how-to-plot-efficiently-a-large-number-of-3d-ellipsoids-with-matplotlib-axes3d
    Parameters
    ----------
    :param mean: Mean of the gaussian and the point where the ellipsoid is to be plotted
    :param cov: Covariance of the gaussian
    """
    if mean.shape[0] == 2:  # 2D Gaussian
        # Find and sort eigenvalues and eigenvectors into descending order
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = eigvals.argsort()[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]
        # The anti-clockwise angle to rotate our ellipse by
        vx, vy = eigvecs[:, 0][0], eigvecs[:, 0][1]
        theta = np.arctan2(vy, vx)
        # Width and height of ellipse to draw
        width, height = 2 * np.sqrt(eigvals)
        ellipse = pat.Ellipse(xy=mean, width=width, height=height,
                       angle=np.degrees(theta),color= 'c', fill=True, alpha=0.1)
        plt.gca().add_patch(ellipse)
    elif mean.shape[0] == 3:    # 3D Gaussian
        # Find the rotation matrix and radii of the axes
        U, s, rotation = np.linalg.svd(cov)
        radii = np.sqrt(s)
        # Calculate cartesian coordinates for the ellipsoid surface
        u = np.linspace(0.0, 2.0 * np.pi, 60)
        v = np.linspace(0.0, np.pi, 60)
        x = radii[0] * np.outer(np.cos(u), np.sin(v))
        y = radii[1] * np.outer(np.sin(u), np.sin(v))
        z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + mean
        plt.gca().plot_wireframe(x, y, z, color='c', rstride=4, cstride=4, alpha=0.1)
    else:
        raise ValueError('Cannot plot Gaussians in more than 3 dimensions')
