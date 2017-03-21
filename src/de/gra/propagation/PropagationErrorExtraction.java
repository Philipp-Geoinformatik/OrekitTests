package de.gra.propagation;

import java.io.File;
import java.util.ArrayList;

import org.hipparchus.geometry.euclidean.threed.FieldRotation;
import org.hipparchus.util.MathArrays.Position;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.SGP4;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.TimeStampedAngularCoordinates;

public class PropagationErrorExtraction {

	public PropagationErrorExtraction() {
		loadOrekitData();
	}

	public ArrayList<SpacePosition> propagateSGP4() throws OrekitException {

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
		
		
//		sgp4.setMasterMode(new OrekitStepHandler() {
//
//			@Override
//			public void handleStep(OrekitStepInterpolator arg0, boolean arg1) throws OrekitException {
//				double x = arg0.getCurrentState().getPVCoordinates().getPosition().getX();
//				double y = arg0.getCurrentState().getPVCoordinates().getPosition().getY();
//				double z = arg0.getCurrentState().getPVCoordinates().getPosition().getZ();
//				double xO = arg0.getCurrentState().getOrbit().getPVCoordinates().getPosition().getX();
//			System.out.println("================================");
////				System.out.println("X " + x);
////				System.out.println("XO" + xO);
//				pos.add(new SpacePosition(x, y, z));
//			}
//		});

		AbsoluteDate d2 = initialDate.shiftedBy(2000);

		SpacecraftState finalstate = sgp4.propagate(initialDate);
		System.out.println(finalstate.getPVCoordinates().getPosition().getX());
		return pos;
	}

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

	public static void main(String[] args) {

		try {
			new PropagationErrorExtraction().propagateSGP4();
		} catch (OrekitException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

}
