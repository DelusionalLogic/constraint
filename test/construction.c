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

		place_points(&drawing, (double[]){1, 1});

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

		place_points(&drawing, (double[]){1, 1});

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
			.index = 0,
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
			.index = 1,
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
			.index = 2,
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

		place_points(&drawing, (double[]){2, 1, 4});

		assert(drawing.error != NULL);
		assert(&drawing.error->result == p3);
	}

	{
		struct drawing drawing = {};

		struct element* origin = insert_cmd(&drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element* r1 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 0,
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
			.index = 1,
		});

		struct element *c2 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = r2,
		});

		struct element *p = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c1,
			.arg2 = c2,
		});

		place_points(&drawing, (double[]){1.0, 2.0});
		assert(drawing.error != NULL);
		assert(&drawing.error->result == p);
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
			.index = 0,
		});

		struct element *c1 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = r1,
		});

		struct element* spacing = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 1,
		});

		struct element *spacing_circle = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = spacing,
		});

		struct element *center2 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = spacing_circle,
			.arg2 = xaxis,
		});

		struct element* r2 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 2,
		});

		struct element *c2 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = center2,
			.arg2 = r2,
		});

		struct element *tangent_point = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c1,
			.arg2 = c2,
		});

		place_points(&drawing, (double[]){1.0, 2.0, 1.0});
		assert(drawing.error == NULL);
		assert(fabs(tangent_point->point.pos[0] - 1.0) < 0.001);
		assert(fabs(tangent_point->point.pos[1] - 0.0) < 0.001);
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
			.index = 0,
		});

		struct element *c1 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = r1,
		});

		struct element* spacing = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 1,
		});

		struct element *spacing_circle = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = origin,
			.arg2 = spacing,
		});

		struct element *center2 = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = spacing_circle,
			.arg2 = xaxis,
		});

		struct element* r2 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 2,
		});

		struct element *c2 = insert_cmd(&drawing, (struct command){
			.op = CMD_CIRCLE_CENTER_RADIUS,
			.hidden = true,
			.result.type = ETYPE_CIRCLE,
			.arg1 = center2,
			.arg2 = r2,
		});

		struct element *p = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_CIRCLE_CIRCLE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = c1,
			.arg2 = c2,
		});

		place_points(&drawing, (double[]){1.0, 5.0, 1.0});
		assert(drawing.error != NULL);
		assert(&drawing.error->result == p);
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

		struct element* angle = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 0,
		});

		struct element *yaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_POINT_LINE_ANGLE,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = origin,
			.arg2 = xaxis,
			.arg3 = angle,
		});

		struct element *intersection = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_LINE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = xaxis,
			.arg2 = yaxis,
		});

		place_points(&drawing, (double[]){M_PI/2});
		assert(drawing.error == NULL);
		assert(fabs(intersection->point.pos[0]) < 0.001);
		assert(fabs(intersection->point.pos[1]) < 0.001);
	}

	{
		struct drawing drawing = {};

		struct element *xaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_X,
			.hidden = true,
			.result.type = ETYPE_LINE,
		});

		struct element* offset1 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 0,
		});

		struct element *line_y1 = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = xaxis,
			.arg2 = offset1,
		});

		struct element* origin = insert_cmd(&drawing, (struct command){
			.op = CMD_ORIGIN,
			.hidden = true,
			.result.type = ETYPE_POINT,
		});

		struct element* angle = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 1,
		});

		struct element *yaxis = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_POINT_LINE_ANGLE,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = origin,
			.arg2 = xaxis,
			.arg3 = angle,
		});

		struct element* offset2 = insert_cmd(&drawing, (struct command){
			.op = CMD_VALUE_INPUT,
			.hidden = true,
			.result.type = ETYPE_VALUE,
			.index = 2,
		});

		struct element *line_x2 = insert_cmd(&drawing, (struct command){
			.op = CMD_LINE_LINE_DISTANCE_PARALLEL,
			.hidden = true,
			.result.type = ETYPE_LINE,
			.arg1 = yaxis,
			.arg2 = offset2,
		});

		struct element *intersection = insert_cmd(&drawing, (struct command){
			.op = CMD_POINT_LINE_LINE,
			.hidden = true,
			.result.type = ETYPE_POINT,
			.arg1 = line_y1,
			.arg2 = line_x2,
		});

		place_points(&drawing, (double[]){1.0, M_PI/2, 2.0});
		assert(drawing.error == NULL);
		assert(fabs(intersection->point.pos[0] - (-2.0)) < 0.001);
		assert(fabs(intersection->point.pos[1] - (-1.0)) < 0.001);
	}
}
