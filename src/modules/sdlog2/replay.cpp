#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149
#define LOG_FORMAT_MSG	0x80
#define LOG_VER_MSG 	130
#define LOG_PARM_MSG 	131
#define LOG_TIME_MSG 	129

#pragma pack(push, 1)
struct log_format_s {
	uint8_t type;
	uint8_t length;		// full packet length including header
	char name[4];
	char format[16];
	char labels[64];
};

struct log_VER_s {
	char arch[16];
	char fw_git[64];
};

struct log_PARM_s {
	char name[16];
	float value;
};

struct log_TIME_s {
	uint64_t t;
};

#pragma pack(pop)

int main(int argc, char *argv[])
{

	// formats
	struct log_format_s formats[100] = {};
	uint8_t crap[300] = {};

	int fd = ::open(argv[1], O_RDONLY);
	bool reached_end = false;



	do {

		uint8_t header[3];

		if (::read(fd, header, 3) != 3) {
			reached_end = true;
		}

		if (header[0] != HEAD_BYTE1 || header[1] != HEAD_BYTE2) {
			printf("bad log header\n");
			return 1;
		}

		if (header[2] == LOG_FORMAT_MSG) {
			struct log_format_s f;
			::read(fd, &f.type, sizeof(f));
			memcpy(&formats[f.type], &f, sizeof(f));
		} else if (header[2] == LOG_PARM_MSG) {
			::read(fd, &crap[0], sizeof(log_PARM_s));
		} else if (header[2] == LOG_VER_MSG) {
			::read(fd, &crap[0], sizeof(log_VER_s));
		} else if (header[2] == LOG_TIME_MSG) {
			::read(fd, &crap[0], sizeof(log_TIME_s));
		} else {
			::read(fd, &crap[0], formats[header[2]].length - 3);

			if (header[2] == 45) {
				float roll = 0;
				memcpy(&roll, &crap[0], 4);
				printf("roll %.5f\n", (double)roll);
			}
		}

	} while (!reached_end);

	return 0;
}