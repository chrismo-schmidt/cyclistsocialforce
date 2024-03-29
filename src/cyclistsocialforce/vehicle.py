"""
Created on Wed Apr 19 16:49:26 2023

Classes defining different vehicle types for the social force model.

Hierarchy:
    Vehicle
     └── Bicycle
         ├── 2DBicycle
         └── InvPendulumBicycle

@author: Christoph Schmidt
"""
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from scipy import interpolate
from math import sqrt

from cyclistsocialforce.utils import (
    limitAngle,
    cart2polar,
    thresh,
    angleDifference,
    toDeg,
    DiffEquation,
)
from cyclistsocialforce.vehiclecontrol import PIDcontroller
from cyclistsocialforce.parameters import (
    VehicleParameters,
    BicycleParameters,
    InvPendulumBicycleParameters,
)
from cyclistsocialforce.vizualisation import BicycleDrawing2D


class Vehicle:
    """Parent class for all vehicle types.

    Provides functions to calculate, update and track destinations based on
    the selected route.
    """

    REQUIRED_PARAMS = ("d_arrived_inter", "hfov")

    def __init__(
        self, s0, userId="unknown", route=(), saveForces=False, params=None
    ):
        """Create a new Vehicle.

        Creates a vehicle object designed to model the planar dynamics of a
        vehicle in 2D space and calculate corresponding social forces according
        to the Cyclist Social Force concept (CSF)

        If a route is provided, an intersection object that owns this
        vehicle will be calculating destinations for crossing an intersection
        according to this route. A route must consist of SUMO edge ids.

        Definition of state variables:
            x: position [m] in the global coordinate frame
            y: position [m] in the global coordinate frame
            theta: heading [rad] relative to the positive x-axis in global
                   frame. Positive angles go counterclockwise.
            v: longitudinal speed [m/s] of the vehicle
            delta: steer angle relative to the heading. Positive angles go
                 counterclockwise.

        Parameters
        ----------
        s0 : List of float
            List containing the initial vehicle state (x, y, theta, v, delta).
        userId : str, optional
            String for identifying this vehicle. Default = ""
        route : list of str, optional
            List of SUMO edge IDs describing the vehicle route. An empty list
            deactivates route following and destinations have to be set
            using vehicle.setDestinations(). Default = []
        saveForces : boolean, optional
            Save a history of the forces experienced by this vehicle.
            Default = False
        params : VehicleParams, optional
            VehicleParams object providing a parameter set for this vehicle.
            When no parameter object is provided, the vehicle is instantiated
            with default parameters.
        """

        # set parameters if not already set by a child.
        if params is None:
            self.params = VehicleParameters()
        elif params != 0:
            assert isinstance(params, VehicleParameters)
            self.params = params

        # time step counter
        self.i = 0

        # vehicle state (x, y, theta, v, delta)
        self.s = np.array(s0, dtype=float)
        self.s[2] = limitAngle(s0[2])
        self.s_names = ("x[m]", "y[m]", "theta[rad]", "v[m/s]", "delta[rad]")

        # trajectory (list of past states)
        self.traj = np.zeros((5, int(30 / self.params.t_s)))
        self.traj[:, 0] = self.s

        self.saveForces = saveForces
        if self.saveForces:
            self.trajF = np.zeros((2, int(30 / self.params.t_s)))

        # vehicle id
        assert isinstance(userId, str), "User ID has to be a string."
        self.id = userId

        # follow a route
        assert isinstance(route, tuple), "Route has to be a tuple"
        assert all(
            isinstance(r, str) for r in route
        ), "Edge IDs in route \
            list have to be str"
        self.follow_route = bool(route)
        self.route = route

        # has a drawing
        self.hasDrawings = [False]

        # next destination and queue of following destination
        self.dest = np.array([s0[0], s0[1], 0.0])
        self.destqueue = np.c_[self.dest[0], self.dest[1], self.dest[2]]
        self.destpointer = 0

        # navigation state
        self.znav = np.array([True, False, False])

    def updateNavState(self, stop):
        """Update the navigation state of the vehicle.

        Calculates the next state of the navigation state machine depending
        on the current distance from the next destination and weather the
        vehicles is tasked to stop there or not.

        Parameters:
        -----------

        stop : bool
            Indicator if the vehicle should stop at the next destination.

        Returns
        -------
        vd : float
            Desired velocity in the current navigation state.

        """

        assert np.any(self.znav), "Invalid state!"

        # break distance safety factor
        k = 1.5

        if self.znav[0]:
            # Distance to stop with gentle-first decceleration profile
            d0 = (
                0.5
                * (self.params.v_max_harddecel**2 - self.s[3] ** 2)
                / self.params.a_desired_default[0]
            )
            d1 = 0.5 * -self.params.v_max_harddecel**2 / self.params.a_max[0]
        else:
            d0 = self.znavparams[1]
            d1 = self.znavparams[2]

        # Distance to destination
        ddest = self.getDestinationDistance()

        # conditions for state transitions
        x = np.zeros(4, dtype=bool)

        x[0] = bool(stop)
        x[1] = ddest <= k * (d0 + d1)
        x[2] = ddest <= self.params.d_arrived_stop
        x[3] = self.s[3] <= self.params.v_max_stop

        # next state
        z = self.znav.copy()
        self.znav[0] = (
            not x[0] or x[0] and not x[1] and (z[0] and not x[2] or z[1])
        )
        self.znav[1] = x[0] and (
            z[0]
            and (not x[2] and x[1] or x[2] and not x[3])
            or z[1]
            and x[1]
            and (not x[2] or not x[3])
        )
        self.znav[2] = x[0] and ((z[0] or z[1]) and x[2] and x[3] or z[2])

        if np.sum(self.znav) != 1:
            print("")
            print("prev state = " + str(z))
            print("next state = " + str(self.znav))
            print("")
            print("stop     = " + str(x[0]))
            print("dstop    = " + str(x[1]))
            print("darrived = " + str(x[2]))
            print("vstop    = " + str(x[3]))
            # raise Exception("Invalid state!")

        # state-dependent variables
        if z[0] and self.znav[1]:
            # z1: [v0, d0, d1]
            self.znavparams = [self.s[3], d0, d1, self.i]
            # print(self.znavparams)
            # print(ddest)

        if self.znav[0]:
            vd = self.params.v_desired_default
        elif self.znav[1]:
            if ddest < k * self.znavparams[2]:
                vd = (
                    self.params.v_max_harddecel
                    / self.znavparams[2]
                    * ddest
                    * 1
                    / k
                )
            else:
                vd = (
                    self.znavparams[0] - self.params.v_max_harddecel
                ) / self.znavparams[1] * (
                    ddest - self.znavparams[2]
                ) * 1 / k + self.params.v_max_harddecel
            # vd = max(vd, self.znavparams[0])
        elif self.znav[2]:
            vd = 0
        else:
            raise Exception("Invalid navigation state")

        return vd, ddest

    def stop(self, stoptype=0, stopdest=None):
        """Make the vehicle come to a halt.

        Provide a stoptype to select how the vehicle should stop.
            0 - At the next destination in the queue. (default)
            1 - As soon as possible (emergency stop).
            2 - At a given location stopdest.

        Parameters
        ----------
        stoptype : int, default = 0
            Type of stop to be performed. Must be one of (0,1,2).
        stopdest : array_like, default = None
            Stop destination (xstop, ystop). Only used if stoptype=2.

        Returns
        -------
        None

        Changelog
        ---------
        v0.1.x  First implementation
        """

        if stoptype == 0:
            self.dest[2] = 1.0
        elif stoptype == 1:
            tstop = abs(self.s[3] / self.params.AMAX[0])
            dstop = 1.1 * (
                self.s[3] * tstop + 0.5 * self.params.AMAX[0] * tstop**2
            )

            xstop = self.s[0] + dstop * np.sin(self.s[2])
            ystop = self.s[1] + dstop * np.cos(self.s[2])

            self.dest = np.array([xstop, ystop, 1.0])
            if self.destpointer > 0:
                self.destpointer -= 1

        elif stoptype == 2:
            self.dest = np.array([stopdest[0], stopdest[1], 1.0])
            if self.destpointer > 0:
                self.destpointer -= 1
        else:
            raise ValueError("Stop type has to be one of [0,1,2].")

    def go(self, gotype=0):
        """Make the vehicle continue after a stop or abort the current stop
        maneuver.

        Provide a gotype to select how the vehicle should behave.
            0 - Keep current destination but do not stop there. (default)
            1 - Go to next destination in queue. Use this for recovery after an
                unscheduled stop (stoptype 1 or 2).

        Parameters
        ----------
        gotype : int, default = 0
            Type of maneuver to be performed. Must be one of (0,1).

        Returns
        -------
        None

        Changelog
        ---------
        v0.1.x  First implementation
        """

        if gotype == 0:
            self.dest[2] = 0.0
        elif gotype == 1:
            self.dest = self.destqueue[self.destpointer, :]

        self.arrived = False

        self.updateDestination()

    def isLastDest(self):
        """Check of the current destination is the last destination in queue."""
        if self.destqueue is None:
            return True
        if self.destpointer + 1 >= np.shape(self.destqueue)[0]:
            return True
        return False

    def updateDestination(self):
        """Check if the road user has reached the current intermediate
        destination and update to the next destination.

        Changelog
        ---------
        v0.0.x  First implementation
        v0.1.x  Added stop destination compatibility

        """
        assert (
            self.destqueue is not None
        ), "Road user does not have a \
            destination queue!"

        dnext = self.getDestinationDistance()

        # save last destination stop property for later
        destprev = self.dest

        # check if road user arrived at a stop destination or is currently
        # stopping for one.
        if np.any(self.znav[1:]):
            return

        # check if road user arrived at an intermediate destination.
        if dnext <= self.params.d_arrived_inter:
            self.destpointer = min(
                self.destpointer + 1, np.shape(self.destqueue)[0] - 1
            )

        # check if we can jump the next destination in the queue
        if self.destpointer < (np.shape(self.destqueue)[0] - 1):
            dnextnext = np.sqrt(
                (self.destqueue[self.destpointer + 1, 0] - self.s[0]) ** 2
                + (self.destqueue[self.destpointer + 1, 1] - self.s[1]) ** 2
            )
            if dnextnext < dnext:
                self.destpointer += 1

        # update destination
        self.dest = self.destqueue[self.destpointer, :]

        # save time step and distance between destinations if the next dest
        # changes to a stopdest.
        if not bool(destprev[2]) and bool(self.dest[2]):
            self.iStopsignal = self.i
            self.dStopsignal = np.sqrt(
                np.sum(np.power(destprev[0:2] - self.dest[0:2], 2))
            )

    def getDestinationDistance(self):
        """Return the distance of the ego vehicle to it's current destination."""
        return np.sqrt(
            np.power(self.dest[0] - self.s[0], 2)
            + np.power(self.dest[1] - self.s[1], 2)
        )

    def setDestinations(self, x, y, stop=None, reset=False):
        """Set the next destinations.

        Use this function to pass a trajectory prototype to the vehicle.

        Parameters
        ----------
        x : np.ndarray of float
            List of the next destination x coordinates. First destination has
            to be at x[0].
        y : np.ndarray of float
            List of the next destination x coordinates. First destination has
            to be at y[0].
        stop : np.ndarray of float, default = np.zeros_like(x)
            Indicator if the cyclist should stop at an intermediate
            destination. 1.0 -> stop, 0.0 -> no stop.
        reset : bool
            If false, the new destinations are appended to the end of the
            queue. If true, the current queue is deleted and the the first
            element of dest becomes the immediate next destination.

        Changelog
        ---------
        v0.0.x  First implementation
        v0.1.x  Added stop flags

        """

        if stop == None:
            stop = np.zeros_like(x)

        if reset or self.destqueue is None:
            self.destqueue = np.c_[x, y, stop]
            self.destpointer = 0
        else:
            self.destqueue = np.vstack((self.destqueue, np.c_[x, y, stop]))

    def setSplineDestinations(self, x, y, npoints, stop=False, reset=False):
        """Calculate and set intermediate destinations according cubic
        spline.

        Parameters
        ----------

        x : array_like
            List of x locations for the spline creation in ascending order.
            The first location should equal the position of the vehicle. Must
            be at least three locations.

        y : array_like
            List of y locations for the spline creation in ascending order.
            The first location should equal the position of the vehicle. Must
            be at least three locations.

        npoints : int
            Number of points of the spline.

        stop : bool, default=False
            If true, the last spline location will the set as as stop
            destination.

        reset : bool, default=False
            If true, the spline overwrites the current destination queue. If
            False, the spline is appended to the current queue.

        """
        assert (
            len(x) >= 3
        ), "Provide at least 3 points to calculate a cubic \
            trajectory prototype"

        x = np.insert(np.array(x), 0, self.s[0])
        y = np.insert(np.array(y), 0, self.s[1])
        tck, u = interpolate.splprep((x, y), s=0.0)
        x_i, y_i = interpolate.splev(np.linspace(0, 1, npoints), tck)

        if stop:
            stop = np.zeros_like(x_i)
            stop[-1] = 1.0
            self.setDestinations(x_i, y_i, stop=stop, reset=reset)
        else:
            self.setDestinations(x_i, y_i, reset=reset)

    def endAnimation(self):
        """End animation of the vehicle

        Set the "animated" property of all graphic object to False to prevent
        them from disappearing once the animation ends.

        Returns
        -------
        None.

        """
        if self.hasDrawings:
            for g in self.ghandles:
                if g is not None:
                    g.set_animated(False)

    def restartAnimation(self):
        """End animation of the vehicle

        Set the "animated" property of all graphic object to False to prevent
        them from disappearing once the animation ends.

        Returns
        -------
        None.

        """
        if self.hasDrawings:
            for g in self.ghandles:
                if g is not None:
                    g.set_animated(True)

    def plot_states(
        self,
        axes: list[Axes] = None,
        states_to_plot: list[bool] = None,
        t_end: float = None,
        **plot_kw,
    ) -> list[Axes]:
        """Plot the state trajectories


        Parameters
        ----------
        axes : list[Axes], optional
            Axes to be plotted in. Must be one axes per requested state to be
            plotted. The default is None and creates a new set of axes.
        states_to_plot : list[bool], optional
            States to be plotted. Must be a list of booleans indicating which
            state should be plotted and which not. The default is None which
            plots all states.
        t_end : float, optional
            Time span of the plot in s. The default plots the full available
            trajectory.
        **kwargs : TYPE
            Parameters to be passed to plt.Axes.plot() for changing the
            appearance of the plotted lines.

        Returns
        -------
        axes : list[axes]
            List of axes the states where plotted in.

        """

        if t_end is None:
            i_end = self.traj.shape[1]
        else:
            i_end = min(int(t_end / self.params.t_s), self.traj.shape[1])

        if states_to_plot is None:
            states_to_plot = [True] * self.s.shape[0]

        if axes is None:
            fig, axes = plt.subplots(np.sum(states_to_plot), 1, sharex=True)
            for ax, name in zip(axes, np.array(self.s_names)[states_to_plot]):
                ax.set_ylabel(name)
            axes[-1].set_xlabel("t[s]")
            axes[-1].legend()

        assert (
            len(states_to_plot) == self.s.shape[0]
        ), f"The parameter \
            states_to_plot needs to have the same number of elements as \
            this vehicle has states ({self.s.shape[0]}). Instead it has \
            {len(states_to_plot)} elements."
        assert len(axes) == np.sum(
            states_to_plot
        ), f"There were {len(axes)} \
            axes provided and plots of {np.sum(states_to_plot)} states \
            requested. Please provide the same number of axes objects in \
            axes as there are TRUE values in states_to_plot!"

        if "label" not in plot_kw:
            plot_kw["label"] = self.id

        for ax, i_s in zip(axes, np.cumsum(states_to_plot)):
            ax.plot(self.traj[i_s - 1, :i_end], **plot_kw)

        if axes[-1].get_legend() is not None:
            axes[-1].get_legend().remove()
        axes[-1].legend()

        return axes

    def plot_forces(
        self,
        axes: list[Axes] = None,
        components_to_plot: list[str] = [
            "direction",
        ],
        t_end: float = None,
        **plot_kw,
    ) -> list[Axes]:
        """
        Plot the trajectory of overall forces.

        Parameters
        ----------
        axes : list[Axes], optional
            Axes to be plotted in. Must be one axes per requested component to
            be plotted. The default is None and creates a new set of axes.
        components_to_plot : list[str], optional.
            List of strings specifying the components to be plotted. Components
            can be any of "magnitude", "direction", "x", "y". The default is
            ['direction',] which plots only the direction of the overall force.
        t_end : float, optional
            Time span of the plot in s. The default plots the full available
            trajectory.
        **kwargs : TYPE
            Parameters to be passed to plt.Axes.plot() for changing the
            appearance of the plotted lines.

        Returns
        -------
        list[Axes]
            List of axes the force components where plotted in.

        """

        if t_end is None:
            i_end = self.traj.shape[1]
        else:
            i_end = int(t_end / self.params.t_s)

        COMPONENTS = ("magnitude", "direction", "x", "y")

        assert (
            all(c in COMPONENTS for c in components_to_plot)
            and len(components_to_plot) > 0
        ), f' components_to_plot may only \
            containt the following strings: "magnitude", "direction", "x", \
            "y". Instead it was {components_to_plot}.'

        if axes is None:
            fig, axes = plt.subplots(len(components_to_plot), 1, sharex=True)
            if not isinstance(axes, np.ndarray):
                axes = [
                    axes,
                ]
            for ax, c in zip(axes, components_to_plot):
                ax.set_ylabel(c)

        assert len(axes) == len(
            components_to_plot
        ), f"There were \
            {len(axes)} axes provided and plots of {len(components_to_plot)} \
            force components requested. Please provide the same number of \
            axes objects in axes as there are str in components_to_plot!"

        if "label" not in plot_kw:
            plot_kw["label"] = self.id

        for ax, c in zip(axes, components_to_plot):
            if c == COMPONENTS[0]:
                y = np.sqrt(
                    self.trajF[0, :i_end] ** 2 + self.trajF[1, :i_end] ** 2
                )
            if c == COMPONENTS[1]:
                y = (360 / (2 * np.pi)) * np.arctan2(
                    self.trajF[1, :i_end], self.trajF[0, :i_end]
                )
            if c == COMPONENTS[2]:
                y = self.trajF[0, :i_end]
            if c == COMPONENTS[3]:
                y = self.trajF[1, :i_end]

            ax.plot(y, **plot_kw)

        if axes[-1].get_legend() is not None:
            axes[-1].get_legend().remove()
        axes[-1].legend()

        return axes


class Bicycle(Vehicle):
    """Parent class for all bicycle types. Child of Vehicle.

    Inherits functions to calculate, update and track destinations based on
    the selected route from Vehicle.

    Provides functions to calculate and update potentials and social forces
    common to all bicycle types.
    Provides funtions to move and control the 2D kinematic bicycle.
    """

    REQUIRED_PARAMS = (
        "l",
        "k_p_v",
        "k_p_delta",
        "a_max",
        "delta_max",
        "v_max_riding",
        "v_desired_default",
        "p_decay",
        "p_0",
        "d_arrived_inter",
        "hfov",
    )

    def __init__(
        self, s0, userId="unknown", route=(), saveForces=False, params=None
    ):
        """

        Parameters
        ----------
        s0 : List of float
            List containing the initial vehicle state (x, y, theta, v, delta).
        userId : str, optional
            String for identifying this vehicle. Default = ""
        route : list of str, optional
            List of SUMO edge IDs describing the vehicle route. An empty list
            deactivates route following and destinations have to be set
            using vehicle.setDestinations(). Default = []
        saveForces : boolean, optional
            Save a history of the forces experienced by this vehicle.
            Default = False
        params : VehicleParams, optional
            VehicleParams object providing a parameter set for this vehicle.
            When no parameter object is provided, the vehicle is instantiated
            with default parameters.
        """

        # set parameters if not already set by a child.
        if params is None:
            self.params = BicycleParameters()
        elif params != 0:
            assert isinstance(params, BicycleParameters)
            self.params = params

        # call super
        Vehicle.__init__(self, s0, userId, route, saveForces, 0)

        self.updateExcentricity()

        # ego repulsive force
        self.F = []

        # has a drawing of a bicycle
        self.hasDrawings = [False] * 8

        self.destspline = None

        # vehicle control inputs and controllers (separate controlers for speed
        # and steer angle)
        self.controlinput = ([], [])
        self.controlsignals = ([], [])
        self.controllers = (
            PIDcontroller(
                self.params.k_p_delta, 0, 0, self.params.t_s, isangle=True
            ),
            PIDcontroller(
                self.params.k_p_v, 0, 0, self.params.t_s, isangle=False
            ),
        )

    def updateExcentricity(self):
        """Update the execentricity of the ego repulsive potential

        The potentials for repulsive force calculations are ellipses with
        speed-depended excentricity. This updates the excentricity according
        the current speed.

        """
        self.e = min(
            np.power(self.s[3] / self.params.v_max_riding[1], 0.1), 0.7
        )

    def calcPotential(self, x, y):
        """Evaluate the repulsive potential of the ego vehicle.

        Evaluates at all locations in x and y.


        Parameters
        ----------
        x : Tuple of floats
            x locations for potential evaluation. len(x) must be equal len(y)
        y : Tuple of floats
            y locations for potential evaluation. len(x) must be equal len(y)


        """

        # coordinate transformations
        x0 = x - self.s[0]  # +.5*np.cos(self.s[2])
        y0 = y - self.s[1]  # -.5*np.sin(self.s[2])

        rho, phi = cart2polar(x0, y0)

        phi0 = angleDifference(phi, self.s[2])
        phi0 = phi - self.s[2]

        # excentricity of elliptic potential according to vehicle speed
        self.updateExcentricity()

        # elliptic potential
        b = (
            (1 / (np.sqrt(1 - np.power(self.e, 2)) * self.params.p_decay))
            * rho
            * (1 - self.e * np.cos(phi0))
        )

        P = self.params.p_0 * np.exp(-b)
        # P[P>self.PMAX] = self.PMAX

        return P

    def calcRepulsiveForce(self, x, y, phi=None):
        """Evaluate the repulsive force of the ego vehicle.

        Evaluates at all locations in x and y.


        Parameters
        ----------
        x : Tuple of floats
            x locations for potential evaluation. len(x) must be equal len(y)
        y : Tuple of floats
            y locations for potential evaluation. len(x) must be equal len(y)

        """

        # coordinate transformations
        x0 = x - self.s[0]
        y0 = y - self.s[1]

        rho, phi = cart2polar(x0, y0)

        phi0 = angleDifference(phi, self.s[2])
        phi0 = phi - self.s[2]

        # calc the potential gradient
        P = self.calcPotential(x, y) / self.params.p_decay

        # force
        Frho0 = P * (
            (1 - self.e * np.cos(phi0)) / np.sqrt(1 - np.power(self.e, 2))
        )
        Fphi0 = P * (
            (self.e * np.sin(phi0)) / np.sqrt(1 - np.power(self.e, 2))
        )

        # transform coordinates back

        Fx = Frho0 * np.cos(phi) - Fphi0 * np.sin(phi)
        Fy = Frho0 * np.sin(phi) + Fphi0 * np.cos(phi)

        return (Fx, Fy)

    def calcDestinationForceField(self, x, y):
        """Calculates force vectors from locations in x, y to the current
        destination.

        Evaluates at all locations in x and y.

        If the road user has a destination queue, this also checks if the
        current intermediate destination has to be updated to the next one.


        Parameters
        ----------
        x : Tuple of floats
            x locations for potential evaluation. len(x) must be equal len(y)
        y : Tuple of floats
            y locations for potential evaluation. len(x) must be equal len(y)

        """
        if self.destqueue is not None:
            self.updateDestination()

        vd, ddest = self.updateNavState(self.dest[2])

        if ddest > 0:
            Fx = -vd * (x - self.dest[0]) / ddest
            Fy = -vd * (y - self.dest[1]) / ddest
        else:
            Fx = 0
            Fy = 0

        if np.isnan(Fx) or np.isnan(Fy):
            print(ddest)
            print(vd)
            print(x)
            print(y)
            raise Exception("Isnan!")

        return (Fx, Fy)

    def calcDestinationForce(self):
        """Calculates force vectors from the current ego vehicle location to
        the current destination.
        """

        return self.calcDestinationForceField(self.s[0], self.s[1])

    def calcDestinationForceHM(self):
        """Calculates the destination force according to Helbing and Molnar"""

        relax = 3

        rx, ry = self.calcDestinationForce()

        r = np.sqrt(rx**2 + ry**2)
        ex = rx / r
        ey = ry / r

        Fx = (1 / relax) * (
            self.params.v_desired_default * ex
            - (self.s[3] * np.cos(self.s[2]))
        )
        Fy = (1 / relax) * (
            self.params.v_desired_default * ey
            - (self.s[3] * np.sin(self.s[2]))
        )

        return (Fx, Fy)

    def control(self, Fx, Fy):
        """Calculate control effort based on the current social force"""

        ddest = self.getDestinationDistance()

        theta = np.arctan2(Fy, Fx)
        v = np.sqrt(np.power(Fx, 2) + np.power(Fy, 2))

        ddest = np.sqrt(
            np.power(self.dest[0] - self.s[0], 2)
            + np.power(self.dest[1] - self.s[1], 2)
        )

        if ddest < 3 and self.isLastDest():
            v = (v / 3) * ddest
        self.controlinput[1].append(v)

        target_angle_ego = angleDifference(self.s[2], theta)

        self.controlinput[0].append(target_angle_ego)

        ddelta = angleDifference(self.s[4], target_angle_ego)
        dv = v - self.s[3]

        odelta = self.controllers[0].step(ddelta)
        a = self.controllers[1].step(dv)

        return a, odelta

    def move(self, a=0, ddelta=0):
        """Propagate the dynamic vehicle model by one step."""
        a = thresh(a, self.params.a_max)

        self.controlsignals[0].append(ddelta)
        self.controlsignals[1].append(a)

        delta = limitAngle(self.s[4] + self.params.t_s * ddelta)
        v = self.s[3] + self.params.t_s * a

        delta = thresh(delta, (-self.params.delta_max, self.params.delta_max))
        v = thresh(v, self.params.v_max_riding)

        theta = self.s[2] + self.params.t_s * v * np.tan(delta) / self.params.l

        theta = limitAngle(theta)

        y = self.s[1] + self.params.t_s * v * np.sin(theta)
        x = self.s[0] + self.params.t_s * v * np.cos(theta)

        # self.s = [x,y,theta,v,delta]
        self.s[0] = x
        self.s[1] = y
        self.s[2] = theta
        self.s[3] = v
        self.s[4] = delta

    def step(self, Fx, Fy):
        """Execute one time step of control and bike kinematics"""
        a, odelta = self.control(Fx, Fy)
        self.move(a, odelta)

        self.i += 1
        self.i = self.i % self.traj.shape[1]

        self.traj[:, self.i] = self.s

    def makeBikeDrawing(
        self,
        ax,
        drawForce=False,
        drawTrajectory=False,
        drawNextDest=False,
        drawDestQueue=False,
        drawPastDest=False,
        drawName=False,
        animated=True,
    ):
        """Create a bicycle drawing using current state.

        Parameters
        ----------

        ax : Axes
            Axes to draw the bicycle in
        drawForce : boolean, optional
            Draw the current destination force. Default = False
        drawTrajectory : boolean, optional
            Draw a solid line showing the past trajectories. Default = False
        drawNextDest : boolean, optional
            Draw a dotted line to the next destiation. Default = False
        drawDestQueue : boolean, optional
            Draw the current destination force. Default = False
        drawPastDest : boolean, optional
            Draw past destinations. Default = False
        drawName : boolean, optional
            Draw the name of the bike. Default = False
        animated : boolean, optional
            Create an animated drawing. Default = True

        """

        self.hasDrawings[0] = True
        self.hasDrawings[1] = drawForce
        self.hasDrawings[3] = drawTrajectory
        self.hasDrawings[4] = drawNextDest
        self.hasDrawings[5] = drawDestQueue
        self.hasDrawings[6] = drawPastDest
        self.hasDrawings[7] = drawName

        # list to save graphic object handles
        self.ghandles = [None] * (15)

        self.drawing = BicycleDrawing2D(ax, self, proj_3d=False)

        # force
        if drawForce:
            self.ghandles[8] = ax.arrow(
                self.s[0],
                self.s[1],
                self.force[0],
                self.force[1],
                linewidth=1,
                head_width=0.4,
                head_length=0.5,
                color=(12.0 / 255, 35.0 / 255, 64.0 / 255),
                animated=animated,
                zorder=3,
            )
            ax.draw_artist(self.ghandles[8])

        # potential
        # REMOVED due to errors (CS, 25-09-2023)

        # trajectory
        if drawTrajectory:
            (self.ghandles[10],) = ax.plot(
                self.traj[0],
                self.traj[1],
                color=(0.0 / 255, 166.0 / 255, 214.0 / 255),
                linewidth=1,
                animated=animated,
            )
            ax.draw_artist(self.ghandles[10])

        # next destination
        if drawNextDest:
            (self.ghandles[11],) = ax.plot(
                (self.s[0], self.dest[0]),
                (self.s[1], self.dest[1]),
                color="gray",
                linewidth=1,
                linestyle="dashed",
                animated=animated,
                zorder=3,
            )
            ax.draw_artist(self.ghandles[11])
            if not drawDestQueue:
                (self.ghandles[12],) = ax.plot(
                    self.dest[0],
                    self.dest[1],
                    marker="x",
                    markersize=5,
                    markeredgecolor="gray",
                    markeredgewidth=2,
                    animated=animated,
                    zorder=3,
                )
                ax.draw_artist(self.ghandles[12])

        # destination queue
        if drawDestQueue:
            # next and past destinations
            if self.destqueue is None:
                (self.ghandles[12],) = ax.plot(
                    self.dest[0],
                    self.dest[1],
                    marker="x",
                    markersize=5,
                    markeredgecolor=(0.0 / 255, 166.0 / 255, 214.0 / 255),
                    markeredgewidth=1,
                    animated=animated,
                    zorder=3,
                )
                ax.draw_artist(self.ghandles[12])
            else:
                (self.ghandles[12],) = ax.plot(
                    self.destqueue[self.destpointer :, 0],
                    self.destqueue[self.destpointer :, 1],
                    linestyle="None",
                    marker="x",
                    markersize=5,
                    markeredgecolor="gray",
                    markeredgewidth=1,
                    animated=animated,
                    zorder=3,
                )
                ax.draw_artist(self.ghandles[12])

        # past destinations
        if drawPastDest and self.destqueue is not None:
            (self.ghandles[13],) = ax.plot(
                self.destqueue[: self.destpointer, 0],
                self.destqueue[: self.destpointer, 1],
                linestyle="None",
                marker="x",
                markersize=5,
                markeredgecolor="gray",
                markeredgewidth=1,
                animated=animated,
                zorder=3,
            )
            ax.draw_artist(self.ghandles[13])

        # name
        if drawName:
            self.ghandles[14] = ax.text(
                self.s[0],
                self.s[1] + 1,
                self.id,
                color="black",
                fontsize=8,
                animated=animated,
                zorder=4,
            )
            ax.draw_artist(self.ghandles[14])

    def updateBikeDrawing(self, ax):
        """Update bicycle drawing according to the current state vector s

        Parameters
        ----------

        ax : Axes
            Axes of the current bicycle drawing.
        """

        assert self.hasDrawings[
            0
        ], "Call makeBikeDrawing() before updateBikeDrawing()!"

        self.drawing.update(self)

        # force
        if self.hasDrawings[1]:
            self.ghandles[8].set_data(
                x=self.s[0],
                y=self.s[1],
                dx=self.force[0] / 2,
                dy=self.force[1] / 2,
            )
            ax.draw_artist(self.ghandles[8])

        # potential
        # REMOVED due to errors.

        # trajectory
        if self.hasDrawings[3]:
            self.ghandles[10].set_data(
                self.traj[0, 0 : self.i], self.traj[1, 0 : self.i]
            )
            ax.draw_artist(self.ghandles[10])

        # next destination
        if self.hasDrawings[4]:
            if self.destspline is not None:
                self.ghandles[11].set_data(
                    (self.destspline[:, 0]),
                    (self.destspline[:, 1]),
                )
            else:
                self.ghandles[11].set_data(
                    (self.s[0], self.dest[0]),
                    (self.s[1], self.dest[1]),
                )
            ax.draw_artist(self.ghandles[11])

            if not self.hasDrawings[4]:
                self.ghandles[12].set_data(self.dest[0], self.dest[1])
                ax.draw_artist(self.ghandles[12])

        # destination queue
        if self.hasDrawings[5]:
            if self.destqueue is None:
                self.ghandles[12].set_data(self.dest[0], self.dest[1])
            else:
                self.ghandles[12].set_data(
                    self.destqueue[self.destpointer :, 0],
                    self.destqueue[self.destpointer :, 1],
                )
            ax.draw_artist(self.ghandles[12])

        # past destinations
        if self.hasDrawings[6]:
            if self.ghandles[13] is not None:
                self.ghandles[13].set_data(
                    self.destqueue[: self.destpointer, 0],
                    self.destqueue[: self.destpointer, 1],
                )
            ax.draw_artist(self.ghandles[13])

        # name
        if self.hasDrawings[7]:
            self.ghandles[14].set_position((self.s[0], self.s[1] + 1))
            ax.draw_artist(self.ghandles[14])

    def endAnimation(self):
        """End animation of the vehicle

        Set the "animated" property of all graphic object to False to prevent
        them from disappearing once the animation ends.

        Returns
        -------
        None.

        """
        super().endAnimation()

        self.drawing.animated = False
        self.drawing.p.animated = False

    def restartAnimation(self):
        """End animation of the vehicle

        Set the "animated" property of all graphic object to False to prevent
        them from disappearing once the animation ends.

        Returns
        -------
        None.

        """
        super().restartAnimation()
        self.drawing.animated = True
        self.drawing.p.animated = True


class TwoDBicycle(Bicycle):
    """Bicycle with 2D kinematics. Child of Bicycle.

    Provides funtions to move and control the 2D kinematic bicycle. Introduces
    new functions to calculate repulsive potentials and a spline-based
    destination force.

    This model was introduced for the conference paper
    Schmidt, C., Dabiri, A., Schulte, F., Happee, R. & Moore, J. (2023).
    Essential Bicycle Dynamics for Microscopic Traffic Simulation:
    An Example Using the Social Force Model. The Evolving Scholar - BMD 2023,
    5th Edition. https://doi.org/10.59490/65037d08763775ba4854da53 and is
    refered to as "2D model" in the publication.
    """

    REQUIRED_PARAMS = (
        "l",
        "k_p_v",
        "k_p_delta",
        "a_max",
        "delta_max",
        "v_max_riding",
        "v_desired_default",
        "p_decay",
        "p_0",
        "d_arrived_inter",
        "hfov",
        "g",
    )

    def __init__(
        self, s0, userId="unknown", route=(), saveForces=False, params=None
    ):
        """Create a new bicycle based on the 2D two-wheeler model.

        TwoDBicycle state variable definition:
             [  x  ]   horizontal position [m]
             [  y  ]   vertical position [m]
        s =  [ phi ]   yaw angle [rad]
             [  v  ]   longitudinal speed [m/s]
             [delta]   steer angle [rad]

         Parameters
         ----------
         s0 : List of float
             List containing the initial vehicle state (x, y, theta, v, delta).
         userId : str, optional
             String for identifying this vehicle. Default = ""
         route : list of str, optional
             List of SUMO edge IDs describing the vehicle route. An empty list
             deactivates route following and destinations have to be set
             using vehicle.setDestinations(). Default = []
         saveForces : boolean, optional
             Save a history of the forces experienced by this vehicle.
             Default = False
         params : VehicleParams, optional
             VehicleParams object providing a parameter set for this vehicle.
             When no parameter object is provided, the vehicle is instantiated
             with default parameters.
        """

        if params is None:
            self.params = InvPendulumBicycleParameters()
        elif params != 0:
            assert isinstance(params, InvPendulumBicycleParameters)
            self.params = params

        Bicycle.__init__(self, s0[0:5], userId, route, saveForces, 0)

        # current state
        #   [  x  ]   horizontal position
        #   [  y  ]   vertical position
        #   [ phi ]   yaw angle
        #   [  v  ]   longitudinal speed
        #   [delta]   steer angle
        self.s = np.array(s0, dtype=float)
        self.s[2] = limitAngle(s0[2])

        # trajectory (list of past states)
        self.traj = np.zeros((5, int(30 / self.params.t_s)))
        self.traj[:, 0] = s0

        self.speed_controller = PIDcontroller(
            self.params.k_p_v, 0, 0, self.params.t_s, isangle=False
        )

    def speed_control(self, vd):
        """Calculate the acceleration as a reaction to the current social
        force.

        Parameters
        ----------

        vd : float
            Desired speed.

        Returns
        -------
        a : float
            Acceleration

        """
        dv = vd - self.s[3]
        a = self.speed_controller.step(dv)

        return a

    def step(self, Fx, Fy):
        """Propagate the full bicycle model by one time step

        Parameters
        ----------
        Fx : float
            X-component of the current social force.
        Fy : float
            y-component of the current social force.
        """

        if self.znav[2]:
            self.s[3:6] = 0
        else:
            a, odelta = super().control(Fx, Fy)
            super().move(a, odelta)

        # Trajectories
        self.i += 1
        self.i = self.i % self.traj.shape[1]

        self.traj[:, self.i] = self.s

        if self.saveForces:
            self.trajF[0, self.i] = Fx
            self.trajF[1, self.i] = Fy

    def calcDestinationForce(self):
        """Calculates force vectors from the current ego vehicle location to
        the current destination.

        Uses spline interpolation to create smooth trajectories between
        multiple intermediate locations. The destination force points
        tangentially to the spline. It's magnitude scales with the spline
        radius.

        If the road user has a destination queue, this also checks if the
        current intermediate destination has to be updated to the next one.

        Overwrites Bicycle.calcDestinationForce()

        TODO: Make look-ahead and number of spline points speed and distance-
        dependend

        Returns
        -------

        Fx : float
            x-component of the desination force
        Fy : float
            y-component of the desination force

        """

        # local parameters. TODO: Move to params class.
        nSplV = 4  # forward number of points in the spline
        tSplB = 3  # backward number of points in the spline
        nSplpnts = 20  # number of interpolated points
        ipred = 3  # look-ahead for normal driving
        ipredlast = 5  # look-ahead for the final destination.

        # update destination and navigation state
        self.updateDestination()
        vd, ddest = self.updateNavState(self.dest[2])

        # For the first step, go into the direction of the current yaw angle.
        if self.i == 0:
            Fx = vd * np.cos(self.s[2])
            Fy = vd * np.sin(self.s[2])
            return Fx, Fy

        # if bike is in navigation state "arrived", return 0
        if self.znav[2]:
            return 0, 0

        # calculate the points for spline interpolation.
        if not self.isLastDest():
            # if the next destination is not the last, use up to nSplV forward
            # destinations to build the spline
            idest = np.arange(
                self.destpointer,
                min(self.destpointer + nSplV, self.destqueue.shape[0]),
                dtype=int,
            )

            x = np.r_[
                self.traj[0, (self.i - 1, self.i)], self.destqueue[idest, 0]
            ]
            y = np.r_[
                self.traj[1, (self.i - 1, self.i)], self.destqueue[idest, 1]
            ]
        else:
            # if the next destination is the last, use trajectory locations
            # from up to tSplB s back.
            # imin = max(0,self.i-tSplB/self.params.t_s)
            # ispl = np.linspace(imin, self.i+1, int(1/self.params.t_s), dtype=int)
            # ispl = np.arange(imin, self.i+1, int(1/self.params.t_s), dtype=int)
            ispl = (
                max(0, self.i - int(1 / self.params.t_s)),
                self.i - 1,
                self.i,
            )
            x = np.r_[self.traj[0, ispl], self.dest[0]]
            y = np.r_[self.traj[1, ispl], self.dest[1]]

        # interpolate
        try:
            tck, u = interpolate.splprep((x, y), s=0.0)
        except TypeError as e:
            print(f"Bike {self.id} has a problem!")
            print(f"isLastDest: {self.isLastDest()}")
            print(f"destqueue: {self.destqueue}")
            print(f" i: {self.i}")
            print(f" ispl: {ispl}")
            print(f"x : {x}")
            print(f"x : {y}")
            raise e
        xs, ys = interpolate.splev(np.linspace(0, 1, nSplpnts), tck)
        dxs, dys = interpolate.splev(np.linspace(0, 1, nSplpnts), tck, der=1)
        d2xs, d2ys = interpolate.splev(np.linspace(0, 1, nSplpnts), tck, der=2)

        self.destspline = np.c_[xs, ys, dxs, dys, d2xs, d2ys]

        # determine indiced of the current position and the prediction position
        # in the spline
        if self.isLastDest():
            i = np.argmin(
                (self.destspline[:, 0] - self.s[0]) ** 2
                + (self.destspline[:, 1] - self.s[1]) ** 2
            )
        else:
            i = 1
        if self.dest[2]:
            iprev = i + ipredlast
        else:
            iprev = i + ipred

        # calculate the destination force based on the spline.
        if iprev < nSplpnts:
            # calculate the radius of the spline at the bike position from the
            # first and second derivative of the parameterized x and y locations
            R = np.sqrt(
                self.destspline[i, 2] ** 2 + self.destspline[i, 3] ** 2
            ) ** 3 / np.abs(
                self.destspline[i, 2] * self.destspline[i, 5]
                - self.destspline[i, 3] * self.destspline[:, 4]
            )

            # calculate the desired speed at the given radius based on a
            # comfortable lean angle of 10 deg.
            thetacomf = 10 * (2 * np.pi / 360)  # approx 10 deg.
            vminstable = 2.5

            v = max(vminstable, np.sqrt(thetacomf * self.params.g * R[i]))
            v = min(v, vd)

            # calculate resulting x and y forces along the spline.
            temp = v / np.sqrt(
                (self.destspline[iprev, 0] - self.destspline[i, 0]) ** 2
                + (self.destspline[iprev, 1] - self.destspline[i, 1]) ** 2
            )
            Fx = temp * (self.destspline[iprev, 0] - self.destspline[i, 0])
            Fy = temp * (self.destspline[iprev, 1] - self.destspline[i, 1])

        else:
            Fx, Fy = super().calcDestinationForce()

        return Fx, Fy

    def calcRepulsiveForce(self, x, y, psi):
        """Calculate the repulsive force exerted by the ego bicycle on other
        road users in the locations (x,y)

        TODO: Move parameters to params class.

        Parameters
        ----------
        x : array of float
            X-locations of other road users.
        y : array of float
            Y-locations of other road users.
        psi : array of float
              Yaw angles of other road users.

        Returns
        -------
        Fx : float
            x-component of the desination force
        Fy : float
            y-component of the desination force

        """

        x0 = self.s[0]
        y0 = self.s[1]
        psi0 = self.s[2]
        V0 = 7
        Vdecay = [15, 0.5]
        b = 1
        a = 10

        psi_rel = psi0 - psi

        # eccentricity
        e = np.sqrt(1 - (b / a) ** 2)

        # taylored potential shape
        # Vdecay, e = calcPotentialParams(10, 0.5, 2, 10)

        # decay
        Vdecay[0] = 0.5 + 5 * np.sin(psi_rel) ** 2
        Vdecay[1] = 0.3 + 4.9 * np.sin(psi_rel) ** 2

        # vrel = ((self.s[3]/5)**0.1 + 10)/11
        e = e - 0.7 * np.sin(psi_rel) ** 2

        # coordinate transformations
        x0 = x - x0
        y0 = y - y0
        rho, phi1 = cart2polar(x0, y0)
        phi = limitAngle(phi1 - psi0)

        cosphi = np.cos(phi)
        sinphi = np.sin(phi)

        # decay
        sigma = Vdecay[0] - Vdecay[1] * np.sqrt((1 - cosphi) / 2)
        dsigm = -Vdecay[1] * np.sqrt((1 + cosphi) / 2) * np.sign(phi) / 2

        # potential
        P = V0 * np.exp(-rho * np.sqrt(1 - (e * cosphi) ** 2) / sigma)

        # force
        Frho = P * np.sqrt(1 - (e * cosphi) ** 2) / sigma
        Fphi = (
            -P
            * (
                (1 - (e * cosphi) ** 2) * dsigm
                - e**2 * sinphi * cosphi * sigma
            )
            / (sigma**2 * np.sqrt(1 - (e * cosphi) ** 2))
        )

        Fx = Frho * np.cos(phi1) - Fphi * np.sin(phi1)
        Fy = Frho * np.sin(phi1) + Fphi * np.cos(phi1)

        F = np.sqrt(Fx**2 + Fy**2)
        Fx = P * Fx / F
        Fy = P * Fy / F

        return Fx, Fy


class InvPendulumBicycle(TwoDBicycle):
    """InvPendulum bicycle based on the inverted pendulum model. Child of
    TwoDBicycle

    Provides funtions to move and control the inverted pendulum bicycle.
    Inherits the functions to calculate repulsive potentials and a spline-based
    destination force.

    This model was introduced for the conference paper
    Schmidt, C., Dabiri, A., Schulte, F., Happee, R. & Moore, J. (2023).
    Essential Bicycle Dynamics for Microscopic Traffic Simulation:
    An Example Using the Social Force Model. The Evolving Scholar - BMD 2023,
    5th Edition. https://doi.org/10.59490/65037d08763775ba4854da53 and is
    refered to as "inverted pendulum model" in the publication.
    """

    REQUIRED_PARAMS = (
        "l_1",
        "l_2",
        "h",
        "m",
        "i_bike_longlong",
        "k_p_v",
        "g",
        "a_max",
        "delta_max",
        "v_max_riding",
        "v_desired_default",
        "p_decay",
        "p_0",
        "d_arrived_inter",
        "hfov",
    )

    def __init__(
        self, s0, userId="unknown", route=(), saveForces=False, params=None
    ):
        """Create a new bicycle based on the inverted pendulum model.

         InvPendulumBicycle state variable definition:
             [  x  ]   horizontal position [m]
             [  y  ]   vertical position [m]
        s =  [ phi ]   yaw angle [rad]
             [  v  ]   longitudinal speed [m/s]
             [delta]   steer angle [rad]
             [theta]   lean angle [rad]

         Parameters
         ----------
         s0 : List of float
             List containing the initial vehicle state (x, y, theta, v, delta).
         userId : str, optional
             String for identifying this vehicle. Default = ""
         route : list of str, optional
             List of SUMO edge IDs describing the vehicle route. An empty list
             deactivates route following and destinations have to be set
             using vehicle.setDestinations(). Default = []
         saveForces : boolean, optional
             Save a history of the forces experienced by this vehicle.
             Default = False
         params : VehicleParams, optional
             VehicleParams object providing a parameter set for this vehicle.
             When no parameter object is provided, the vehicle is instantiated
             with default parameters.

        """

        if params is None:
            self.params = InvPendulumBicycleParameters()
        elif params != 0:
            assert isinstance(params, InvPendulumBicycleParameters)
            self.params = params

        TwoDBicycle.__init__(self, s0[0:5], userId, route, saveForces, 0)

        # current state
        #   [  x  ]   horizontal position
        #   [  y  ]   vertical position
        #   [ phi ]   yaw angle
        #   [  v  ]   longitudinal speed
        #   [delta]   steer angle
        #   [theta]   lean angle
        self.s = np.array(s0, dtype=float)
        self.s[2] = limitAngle(s0[2])
        self.s[5] = limitAngle(s0[5])
        self.s_names = (
            "x[m]",
            "y[m]",
            "phi[rad]",
            "v[m/s]",
            "delta[rad]",
            "theta[rad]",
        )

        # trajectory (list of past states)
        self.traj = np.zeros((6, int(30 / self.params.t_s)))
        self.traj[:, 0] = s0

        # Model dynamics in Z domain.
        (
            ab_r1,
            ab_r2_delta,
            ab_theta,
            ab_psi,
        ) = self.params.update_dynamic_params(self.s[3])

        self.dynamics_r1 = DiffEquation(ab_r1)
        self.dynamics_r2_delta = DiffEquation(
            ab_r2_delta, th=self.params.delta_max
        )
        self.dynamics_theta = DiffEquation(ab_theta)
        self.dynamics_psi = DiffEquation(ab_psi, y=s0[2] * np.ones(1))

        # state machine
        # TODO: This needs to be tested and improved
        self.zrid = np.zeros((2), dtype=bool)
        if s0[3] < self.params.v_max_walk:
            self.zrid[1] = True
        else:
            self.zrid[0] = True

    def speed_control(self, vd):
        """Calculate the acceleration as a reaction to the current social
        force.

        Parameters
        ----------

        vd : float
            Desired speed.

        Returns
        -------
        a : float
            Acceleration


        """
        dv = vd - self.s[3]
        a = self.speed_controller.step(dv)

        return a

    def step_yaw(self, Fx, Fy):
        """Propagate the yaw dynamics by one time step.

        Parameters
        ----------
        Fx : float
            X-component of the current social force.
        Fy : float
            y-component of the current social force.

        Returns
        -------
        s : list of floats
            Next yaw angle psi, steer angle delta and, lean angle theta of the
            bicycle given as s = (psi, delta, theta)

        """

        # update LTI parameters with current speed
        (
            ab_r1,
            ab_r2_delta,
            ab_theta,
            ab_psi,
        ) = self.params.update_dynamic_params(self.s[3])

        self.dynamics_r1.update(ab_r1)
        self.dynamics_r2_delta.update(ab_r2_delta)
        self.dynamics_theta.update(ab_theta)
        self.dynamics_psi.update(ab_psi)

        # absolute force angle
        psi_d = np.arctan2(Fy, Fx)

        # calculate steer angle for stabilization
        theta_d = self.dynamics_r1.step(angleDifference(self.s[2], psi_d))
        delta = self.dynamics_r2_delta.step(theta_d - self.s[5])
        theta = self.dynamics_theta.step(delta)
        psi = limitAngle(self.dynamics_psi.step(delta))

        return (psi, delta, theta)

    def step_pos(self, Fx, Fy):
        """Propagate the speed dynamics by one time step and integrate
        to calculate the position.

        Parameters
        ----------
        Fx : float
            X-component of the current social force.
        Fy : float
            y-component of the current social force.

        Returns
        -------
        s : list of floats
            Next position and speed of the
            bicycle given as s = (x, y, v)

        """

        vd = sqrt(Fx**2 + Fy**2)

        # print(vd)
        a = self.speed_control(vd)
        # print(a)
        a = thresh(a, self.params.a_max)
        v = self.s[3] + self.params.t_s * a
        v = thresh(v, self.params.v_max_riding)
        # print(v)
        y = self.s[1] + self.params.t_s * v * np.sin(self.s[2])
        x = self.s[0] + self.params.t_s * v * np.cos(self.s[2])

        return (x, y, v)

    def step(self, Fx, Fy):
        """Propagate the full bicycle model by one time step

        Parameters
        ----------
        Fx : float
            X-component of the current social force.
        Fy : float
            y-component of the current social force.
        """

        # Riding state machine
        self.updateRidingState()

        #
        if self.znav[2]:
            self.s[3:6] = 0
        else:
            if self.zrid[0]:
                self.s[[0, 1, 3]] = self.step_pos(Fx, Fy)
                self.s[[2, 4, 5]] = self.step_yaw(Fx, Fy)
            else:
                self.s[3] = self.params.v_max_walk

                # for walking, use the 2D bicycle dynamics
                a, odelta = super().control(Fx, Fy)
                super().move(a, odelta)

                # pass bicycle states to the inverted pendulum dynamics even
                # while walking to prevent errors in the transition.
                self.dynamics_psi.setOutput(self.s[2])
                self.dynamics_psi.setInput(self.s[4])
                self.dynamics_r2_delta.setOutput(self.s[4])
                self.dynamics_r2_delta.setInput(0)

                # bicycle is upright.
                self.s[5] = 0

                # print(f"{self.id} is walking.")

        # counter and trajectories.
        self.i += 1
        self.i = self.i % self.traj.shape[1]

        self.traj[:, self.i] = self.s

        if self.saveForces:
            self.trajF[0, self.i] = Fx
            self.trajF[1, self.i] = Fy

    def updateRidingState(self):
        """Update the riding state of the bicycle

        Changes the riding state between "riding" and "walking" depending on
        the current speed and the steer angle.
        """

        cvwalk = self.s[3] < self.params.v_max_walk

        imin = max(0, int(self.i - 1 / self.params.t_s))

        cdelta = np.all(
            -self.params.delta_max_walk < self.traj[4, imin : self.i + 1]
        ) and np.all(
            self.params.delta_max_walk > self.traj[4, imin : self.i + 1]
        )

        self.zrid[0] = not cvwalk and (self.zrid[1] and cdelta or self.zrid[0])
        self.zrid[1] = not self.zrid[0]
