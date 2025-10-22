#include "cad/construction.h"

int main(int argc, char *argv[]) {
	{
		struct drawing drawing = {};

		struct element* origin = insert_cmd(&drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element *xaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});
		
		struct element* r1 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *c1 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = r1,
		});

		struct element *p2 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c1,
			.arg2 = xaxis,
		});


		struct element* r2 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *c2 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = p2,
			.arg2 = r2,
		});

		struct element *p3 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c1,
			.arg2 = c2,
		});

		execute_drawing(&drawing, (double[]){1, 1});

		assert(drawing.error == NULL);
		assert(p3->point.pos[0] == 0.5 && fabs(p3->point.pos[1] - 0.866025) < 0.001);
	}

	{
		struct drawing drawing = {};

		struct element* origin = insert_cmd(&drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element *xaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});
		
		struct element* r1 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *c1 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = r1,
		});

		struct element *p2 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c1,
			.arg2 = xaxis,
		});


		struct element* r2 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *c2 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = p2,
			.arg2 = r2,
		});

		struct element *p3 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c2,
			.arg2 = c1,
		});

		execute_drawing(&drawing, (double[]){1, 1});

		assert(drawing.error == NULL);
		assert(p3->point.pos[0] == 0.5 && fabs(p3->point.pos[1] - (-0.866025)) < 0.001);
	}

	{
		struct drawing drawing = {};

		struct element* origin = insert_cmd(&drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element *xaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});
		
		struct element* rspace = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *cspace = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = rspace,
		});

		struct element *p2 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = cspace,
			.arg2 = xaxis,
		});

		struct element* r1 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *c1 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = r1,
		});

		struct element* r2 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
		});

		struct element *c2 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = p2,
			.arg2 = r2,
		});

		struct element *p3 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c2,
			.arg2 = c1,
		});

		execute_drawing(&drawing, (double[]){2, 1, 4});

		assert(&drawing.error != NULL);
		assert(&drawing.error->result == p3);
	}
}
