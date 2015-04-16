#include "SailingRobot.h"



int main(int argc, char *argv[]) {

	printf("Sailing robot example\n");

	SailingRobot sr;
	printf("CONFIG");
	std::string path, db, errorLog;
	if (argc < 2) {
		path = "";
		db = "asr.db";
		errorLog = "errors.log";
	} else {
		path = std::string(argv[1]);
		db = "/asr.db";
		errorLog = "/errors.log";
	}
	printf("END");
	try {
		printf("INIT");
		sr.init(path, db, errorLog);
		printf("RUN");
		sr.run();
	} catch (const char * e) {
		printf("ERR");
		sr.shutdown();
		return 1;
	}
	sr.shutdown();
	printf("END");
	return 0;
}
