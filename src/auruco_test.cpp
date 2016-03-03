int main (int argc, char **argv)
{

	try
	{
		CameraParameters CamParam;
		MarkerDetector MDetector;
		vector<Marker> Markers;
		CamParam.readFromXMLFile(argv[2]);
		
