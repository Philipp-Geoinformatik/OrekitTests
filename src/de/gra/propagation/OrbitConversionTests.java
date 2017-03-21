package de.gra.propagation;

import java.io.File;
import java.util.ArrayList;
import java.util.Locale;

import org.hipparchus.ode.nonstiff.AdaptiveStepsizeIntegrator;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.estimation.measurements.EstimatedMeasurement;
import org.orekit.estimation.measurements.PV;
import org.orekit.forces.ForceModel;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.SGP4;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.TimeStampedAngularCoordinates;

/**
 * 
 * @author Philipp Grashorn
 *
 */
public class OrbitConversionTests {

	/**
	 * 
	 */
	private void loadOrekitData() {

		File orekitData = new File("resources/orekit-data");
		DataProvidersManager manager = DataProvidersManager.getInstance();
		try {
			manager.addProvider(new DirectoryCrawler(orekitData));
		} catch (OrekitException e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 */
	private void run() {
		loadOrekitData();
		SpacecraftState state = createSpacecraftState();
		try {
			propagateNumerical(state);
		} catch (OrekitException e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 * 
	 */
	public SpacecraftState createSpacecraftState() {

		// Inertial frame
		Frame inertialFrame = FramesFactory.getEME2000();
		// Initial date
		TimeScale utc = null;
		try {
			utc = TimeScalesFactory.getUTC();
		} catch (OrekitException e1) {
			e1.printStackTrace();
		}
		AbsoluteDate initialDate = new AbsoluteDate(2004, 01, 01, 23, 30, 00.000, utc);
		// Central attraction coefficient
		double mu = 3.986004415e+14;
		// Initial orbit
		double a = 24396159; // semi major axis in meters
		double e = 0.72831215; // eccentricity
		double i = FastMath.toRadians(7); // inclination
		double omega = FastMath.toRadians(180); // perigee argument
		double raan = FastMath.toRadians(261); // right ascention of ascending
												// node
		double lM = 0; // mean anomaly
		Orbit initialOrbit = new KeplerianOrbit(a, e, i, omega, raan, lM, PositionAngle.MEAN, inertialFrame,
				initialDate, mu);
		// Initial state definition
		SpacecraftState initialState = null;
		try {
			initialState = new SpacecraftState(initialOrbit);
		} catch (OrekitException e1) {
			e1.printStackTrace();
		}
		System.out.println("Created SpacecraftSate");
		return initialState;
	}

	/**
	 * 
	 * @param initialState
	 * @throws OrekitException
	 */
	public void propagateNumerical(SpacecraftState initialState) throws OrekitException {
		// Adaptive step integrator
		// with a minimum step of 0.001 and a maximum step of 1000
		double minStep = 0.001;
		double maxstep = 1000.0;
		double positionTolerance = 10.0;
		OrbitType propagationType = OrbitType.KEPLERIAN;
		double[][] tolerances = NumericalPropagator.tolerances(positionTolerance, initialState.getOrbit(),
				propagationType);

		AdaptiveStepsizeIntegrator integrator = new DormandPrince853Integrator(minStep, maxstep, tolerances[0],
				tolerances[1]);
		// We set up the integrator, and force it to use Keplerian parameters
		// for propagation.
		NumericalPropagator propagator = new NumericalPropagator(integrator);
		propagator.setOrbitType(propagationType);
		// A force model, reduced here to a single perturbing gravity field, is
		// taken into account.\ More details on force models can be found in the
		// forces section of the library architecture documentation.
		NormalizedSphericalHarmonicsProvider provider = null;
		provider = GravityFieldFactory.getNormalizedProvider(10, 10);
		ForceModel holmesFeatherstone = new HolmesFeatherstoneAttractionModel(
				FramesFactory.getITRF(IERSConventions.IERS_2010, true), provider);
		// adding force model to the propagator
		propagator.addForceModel(holmesFeatherstone);
		// The propagator operating mode is set to master mode with fixed step
		// and a TutorialStepHandler which implements the interface
		// OrekitFixedStepHandler in order to fulfill the handleStep method to
		// be called within the loop. For the purpose of this tutorial, the
		// handleStep method will just print the current state at the moment.

		propagator.setMasterMode(60., new OrekitFixedStepHandler() {
			@Override
			public void handleStep(SpacecraftState currentState, boolean isLast) throws OrekitException {
				KeplerianOrbit o = (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(currentState.getOrbit());

				PV pv = new PV(currentState.getDate(), currentState.getPVCoordinates().getPosition(),
						currentState.getPVCoordinates().getVelocity(), .10, .10, 1000);

				System.out.format(Locale.US, "%s %12.3f %10.8f %10.6f %10.6f %10.6f %10.6f%n", currentState.getDate(),
						o.getA(), o.getE(), FastMath.toDegrees(o.getI()), FastMath.toDegrees(o.getPerigeeArgument()),
						FastMath.toDegrees(o.getRightAscensionOfAscendingNode()),
						FastMath.toDegrees(o.getTrueAnomaly()));
				if (isLast) {
					System.out.println("this was the last step ");
					System.out.println();
				}
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});
		// Then, the initial state is set for the propagator:
		propagator.setInitialState(initialState);
		// Finally, the propagator is just asked to propagate, from the initial
		// state, for a given duration.
		SpacecraftState finalState = propagator.propagate(new AbsoluteDate(initialState.getDate(), 630.));
	}

	/**
	 * @throws OrekitException 
	 * 
	 */
	void propagateSGP4() throws OrekitException {
		AttitudeProvider attitudeProvider = new AttitudeProvider() {

			@Override
			public Attitude getAttitude(PVCoordinatesProvider arg0, AbsoluteDate arg1, Frame arg2)
					throws OrekitException {
				Frame referenceFrame = FramesFactory.getTEME();
				Object m;
				Object threshold;
				TimeStampedAngularCoordinates orientation = new TimeStampedAngularCoordinates(arg1,
						arg0.getPVCoordinates(arg1, referenceFrame), arg0.getPVCoordinates(arg1, referenceFrame));
				// TODO Auto-generated method stub
				return new Attitude(referenceFrame, orientation);
			}
		};
		double mass = 100;
		// TLE ISS
		String line1 = "1 25544U 98067A   17078.61187328  .00001386  00000-0  28128-4 0  9992";
		String line2 = "2 25544  51.6421 124.8111 0007023 313.4571 184.1549 15.54225252 47821";
		TLE initialTLE = new TLE(line1, line2);
		// SGP4 Model
		SGP4 sgp4 = new SGP4(initialTLE, attitudeProvider, mass);

		TimeScale utc = null;
		try {
			utc = TimeScalesFactory.getUTC();
		} catch (OrekitException e1) {
			e1.printStackTrace();
		}
		AbsoluteDate initialDate = new AbsoluteDate(2004, 01, 01, 23, 30, 00.000, utc);
		ArrayList<SpacePosition> pos = new ArrayList<>();
		
		sgp4.setSlaveMode();

		AbsoluteDate d2 = initialDate.shiftedBy(2000);

		SpacecraftState finalstate = sgp4.propagate(initialDate);
		for (int i = 0; i < 360; i+=60) {
			
		}
		System.out.println(finalstate.getPVCoordinates().getPosition().getX());
		
		
	}

	void convertPropagator(SpacecraftState initialState) throws OrekitException {
		// Adaptive step integrator
		// with a minimum step of 0.001 and a maximum step of 1000
		double minStep = 0.001;
		double maxstep = 1000.0;
		double positionTolerance = 10.0;
		OrbitType propagationType = OrbitType.KEPLERIAN;
		double[][] tolerances = NumericalPropagator.tolerances(positionTolerance, initialState.getOrbit(),
				propagationType);

		AdaptiveStepsizeIntegrator integrator = new DormandPrince853Integrator(minStep, maxstep, tolerances[0],
				tolerances[1]);
		// We set up the integrator, and force it to use Keplerian parameters
		// for propagation.
		NumericalPropagator propagator = new NumericalPropagator(integrator);

	}

	/*
	 * 
	 */
	public static void main(String[] args) {
		new OrbitConversionTests().run();
	}
}
