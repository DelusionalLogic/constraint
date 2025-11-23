#include "cad/svg.h"

#define SIGNOF(x) ((typeof(x))((x)>0) - ((x)<0))

void plot_line_between_style(struct point p1, struct point p2, enum LineStyle style) {
	char *style_str;
	switch(style) {
		case LSTYLE_NORMAL:
			style_str = "stroke=\"black\" stroke-width=\"0.2\"";
			break;
		case LSTYLE_CONSTRUCTION:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-dasharray=\"0.7,0.2\" stroke-opacity=\"0.3\"";
			break;
		case LSTYLE_INDICATOR:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-opacity=\"0.3\"";
			break;
		case LSTYLE_INDICATOR_INLINE:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-opacity=\"0.3\" marker-start=\"url(#triangle)\" marker-end=\"url(#triangle)\"";
			break;
	}

        printf("<line %s x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" />\n", style_str, p1.pos[0], -p1.pos[1], p2.pos[0], -p2.pos[1]);
}

void plot_line_between(struct point p1, struct point p2) {
	plot_line_between_style(p1, p2, LSTYLE_NORMAL);
}

void plot_arc_between_style(struct point c, struct point p1, struct point p2, enum LineStyle style) {
	char *style_str;
	switch(style) {
		case LSTYLE_NORMAL:
			style_str = "stroke=\"black\" stroke-width=\"0.2\"";
			break;
		case LSTYLE_CONSTRUCTION:
			abort();
			break;
		case LSTYLE_INDICATOR_INLINE:
		case LSTYLE_INDICATOR:
			style_str = "stroke=\"blue\" stroke-width=\"0.1\" stroke-opacity=\"0.3\"";
			break;
	}

	bool above_pi;
	{
		vec2 p1l;
		glm_vec2_sub(p1.pos, c.pos, p1l);
		glm_vec2_normalize(p1l);
		vec2 p2l;
		glm_vec2_sub(p2.pos, c.pos, p2l);
		glm_vec2_normalize(p2l);

		glm_vec2_copy((vec2){-p2l[1], p2l[0]}, p2l);

		above_pi = glm_vec2_dot(p1l, p2l) < 0;
	}

	// @COML: There's something missing here about which side of the arc we
	// want. I think we can figure that out from the relation of the center and
	// the points
	vec2 t;
	glm_vec2_sub(c.pos, p1.pos, t);
	double r = glm_vec2_norm(t);
	printf("<path %s d=\"M %f %f A %f %f 0 %d 0 %f %f\" fill=\"none\" />\n", style_str, p2.pos[0], -p2.pos[1], r, r, above_pi, p1.pos[0], -p1.pos[1]);
}

void plot_arc_between(struct point c, struct point p1, struct point p2) {
	plot_arc_between_style(c, p1, p2, LSTYLE_NORMAL);
}

void plot_text(struct point p, double angle, char* str) {
	printf("<text text-anchor=\"middle\" dominant-baseline=\"central\" transform=\"translate(%f, %f) scale(1, -1) rotate(%f) scale(1, -1)\" font-size=\"0.75\">%s</text>\n", p.pos[0], -p.pos[1], angle * (180.0/M_PI), str);
}

void plot_angle(struct line l1, struct line l2, double theta, struct point *intersect, struct point *p1, struct point *p2) {
	line_line_intersect(l1, l2, intersect);

	struct circle c = { .radius = 1.3 };
	glm_vec2_copy(intersect->pos, c.center);

	circle_line_intersect(c, l1, 0, p1);
	circle_line_intersect(c, l2, 0, p2);

	plot_arc_between_style(*intersect, *p2, *p1, LSTYLE_INDICATOR);

	vec2 l1v;
	vec2 l2v;
	glm_vec2_normalize_to(l1.norm, l1v);
	glm_vec2_normalize_to(l2.norm, l2v);

	// Label
	glm_vec2_negate(l2v);

	vec2 x;
	glm_vec2_add(l1v, l2v, x);
	glm_vec2_normalize(x);

	struct point label_point;
	glm_vec2_copy(intersect->pos, label_point.pos);
	glm_vec2_muladds(x, c.radius + TEXT_OFFSET, label_point.pos);

	double dot = x[0];
	double det = x[1];

	double angle = atan2(det, dot);

	char buf[512];
	snprintf(buf, sizeof(buf), "%.1f°", theta);
	plot_text(label_point, angle - M_PI/2, buf);
}

double project_point_to_line_distance(struct point p, struct line l) {
	vec2 offset = {-p.pos[0], -p.pos[1]};
	double c = glm_vec2_dot(l.norm, offset);

	double det = (l.norm[0] * p.pos[1]) - (l.norm[1] * p.pos[0]);
	det = -det;

	// @HACK There's a rounding error here that can cause d1 to end up
	// negative. Just take the abolute value of it to get around that.
	double d1 = fabs(glm_vec2_norm2(p.pos) - pow(c, 2)/glm_vec2_norm2(l.norm));
	assert(d1 >= 0.0);
	double m1 = sqrt(d1 / glm_vec2_norm2(l.norm));

	return SIGNOF(det) * m1;
}

void extend_line_to(struct component* c, struct point *p) {
	assert(c->type == COM_LINE);

	double val = project_point_to_line_distance(*p, c->e->line);
	if(!c->min_max_init) {
		c->min_max_init = true;
		c->min = val;
		c->max = val;
	} else {
		c->min = fmin(c->min, val - 0.1);
		c->max = fmax(c->max, val + 0.1);
	}
}

static void plot_distance_indicator(struct point p1, struct point p2, double distance, bool offset) {
	vec2 dir;
	glm_vec2_sub(p2.pos, p1.pos, dir);
	glm_vec2_normalize(dir);
	vec2 norm = {-dir[1], dir[0]};

	struct point start;
	struct point end;
	{
		if(offset) {
			glm_vec2_add(p1.pos, norm, start.pos);
			glm_vec2_add(p2.pos, norm, end.pos);
		} else {
			glm_vec2_copy(p1.pos, start.pos);
			glm_vec2_copy(p2.pos, end.pos);
		}
		plot_line_between_style(start, end, offset ? LSTYLE_INDICATOR : LSTYLE_INDICATOR_INLINE);
	}

	if(offset) {
		// The little wings to highlight the ends
		vec2 tip = {.3, .3};
		glm_vec2_mul(norm, tip, tip);
		{
			struct point p1;
			struct point p2;
			glm_vec2_add(start.pos, tip, p1.pos);
			glm_vec2_sub(start.pos, tip, p2.pos);
			plot_line_between_style(p1, p2, LSTYLE_INDICATOR);
		}
		{
			struct point p1;
			struct point p2;
			glm_vec2_add(end.pos, tip, p1.pos);
			glm_vec2_sub(end.pos, tip, p2.pos);
			plot_line_between_style(p1, p2, LSTYLE_INDICATOR);
		}
	}

	// The text
	{
		struct point p;
		glm_vec2_lerp(start.pos, end.pos, 0.5, p.pos);

		glm_vec2_muladds(norm, TEXT_OFFSET, p.pos);

		double angle = atan2(dir[1], dir[0]);

		// Flip upside down labels
		if(angle > M_PI/2) {
			glm_vec2_muladds(norm, 0.1, p.pos);
			angle -= M_PI;
		}
		if(angle < -M_PI/2) {
			glm_vec2_muladds(norm, 0.1, p.pos);
			angle += M_PI;
		}

		assert(angle >= -M_PI);
		assert(angle <=  M_PI);

		char buf[512];
		snprintf(buf, sizeof(buf), "%.1f u", distance);
		plot_text(p, angle, buf);
	}
}

void line_distance_to_point(struct line l, double d, struct point *p) {
	glm_vec2_zero(p->pos);
	glm_vec2_muladds(l.norm, l.C, p->pos);
	glm_vec2_divs(p->pos, glm_vec2_norm2(l.norm), p->pos);
	glm_vec2_negate(p->pos);

	vec2 perp = {l.norm[1], -l.norm[0]};
	glm_vec2_muladds(perp, d, p->pos);
}

void draw_constraints(struct constraints *constraints) {
	for(size_t i = 0; i < constraints->length; i++) {
		struct constraint *constraint = &constraints->elements[i];
		if(!constraint->used) continue;

		switch(constraint->type) {
			case CT_POINT_POINT_DISTANCE: {
				assert(constraint->c1->type == COM_POINT);
				assert(constraint->c2->type == COM_POINT);
				if(constraint->v == 0.0) continue;

				// @COMPL: It would be nice to combine the indicator and
				// construction line if there's no topology line there
				plot_line_between_style(constraint->c1->e->point, constraint->c2->e->point, LSTYLE_CONSTRUCTION);
				plot_distance_indicator(constraint->c1->e->point, constraint->c2->e->point, constraint->v, true);
			} break;
			case CT_LINE_LINE_ANGLE: {
				assert(constraint->c1->type == COM_LINE);
				assert(constraint->c2->type == COM_LINE);

				// Check if it's more likely a "parallel" constraint
				if(fabs(sin(constraint->v)) < 0.1) continue;

				struct component *l1 = constraint->c1;
				struct component *l2 = constraint->c2;
				// Negative angles are counterclockwise, we have to swap
				// the arguments to get proper rendering.
				if(constraint->v < 0) {
					l1 = constraint->c2;
					l2 = constraint->c1;
				}

				struct point p1;
				struct point p2;
				struct point intersect;
				plot_angle(l1->e->line, l2->e->line, constraint->v, &intersect, &p1, &p2);

				extend_line_to(constraint->c1, &intersect);
				extend_line_to(constraint->c2, &intersect);
				extend_line_to(l1, &p1);
				extend_line_to(l2, &p2);
			} break;
			case CT_POINT_LINE_DISTANCE: {
				struct component *line;
				struct component *point;
				if(constraint->c1->type == COM_LINE) {
					line = constraint->c1;
					point = constraint->c2;
				} else {
					line = constraint->c2;
					point = constraint->c1;
				}
				assert(point->type == COM_POINT);
				assert(line->type == COM_LINE);

				if(constraint->v > 0) {
					double dist = project_point_to_line_distance(point->e->point, line->e->line);
					struct point closest;
					line_distance_to_point(line->e->line, dist, &closest);

					// @COMPL: It would be nice to combine the indicator and
					// construction line if there's no topology line there
					// plot_line_between_style(closest, point->e->point, LSTYLE_CONSTRUCTION);
					plot_distance_indicator(point->e->point, closest, constraint->v, false);
				}

				extend_line_to(line, &point->e->point);
			} break;
			case CT_END: abort();
		}
	}

	for(size_t i = 0; i < constraints->length; i++) {
		struct constraint *constraint = &constraints->elements[i];
		if(!constraint->used) continue;

		switch(constraint->type) {
			case CT_POINT_POINT_DISTANCE: {
			} break;
			case CT_LINE_LINE_ANGLE: {
				assert(constraint->c1->type == COM_LINE);
				assert(constraint->c2->type == COM_LINE);

				struct point p1;
				struct point p2;
				if(!constraint->c1->drawn) {
					line_distance_to_point(constraint->c1->e->line, constraint->c1->min, &p1);
					line_distance_to_point(constraint->c1->e->line, constraint->c1->max, &p2);
					plot_line_between_style(p1, p2, LSTYLE_CONSTRUCTION);
					constraint->c1->drawn = true;
				}

				if(!constraint->c2->drawn) {
					line_distance_to_point(constraint->c2->e->line, constraint->c2->min, &p1);
					line_distance_to_point(constraint->c2->e->line, constraint->c2->max, &p2);
					plot_line_between_style(p1, p2, LSTYLE_CONSTRUCTION);
					constraint->c2->drawn = true;
				}
			} break;
			case CT_POINT_LINE_DISTANCE:
				break;
			case CT_END: abort();
		}
	}
}

void draw_topology(struct topology *topo) {
	struct component *head = NULL;
	for(size_t i = 0; i < topo->length; i++) {
		struct topology_elem *cur = &topo->elements[i];
		switch(cur->cmd.op) {
			case TOPO_MOVETO:
				cur++;
				if(cur->arg.c->e != NULL) head = cur->arg.c;
				break;
			case TOPO_LINETO:
				cur++;
				if(cur->arg.c->e != NULL) {
					plot_line_between(head->e->point, cur->arg.c->e->point);
					head = cur->arg.c;
				}
				break;
			case TOPO_ARCTO:
				cur++;
				if(cur->arg.c->e != NULL && (cur+1)->arg.c->e != NULL) {
					plot_arc_between(cur->arg.c->e->point, (cur+1)->arg.c->e->point, head->e->point);
					head = (cur+1)->arg.c;
				}
				break;
			case TOPO_END:
				abort();
		}
	}
}

void begin_drawing() {
	printf("<svg version=\"1.1\" viewBox=\"-50 -50 100 100\" width=\"1200\" height=\"1200\" xmlns=\"http://www.w3.org/2000/svg\">\n");
	printf("<defs>\n");
    printf("\t<marker id=\"triangle\" viewBox=\"0 0 10 10\" refX=\"10\" refY=\"5\" markerUnits=\"strokeWidth\" markerWidth=\"6\" markerHeight=\"6\" orient=\"auto-start-reverse\">\n");
    printf("\t\t<path d=\"M 0 0 L 10 5 L 0 10 z\" fill=\"blue\" opacity=\"0.3\" />\n");
    printf("\t</marker>\n");
	printf("</defs>\n");

	// Axis lines
	printf("<line x1=\"-1000\" y1=\"0\" x2=\"1000\" y2=\"0\" stroke=\"black\" stroke-width=\"0.1\" stroke-opacity=\"0.4\" />\n");
	printf("<line y1=\"-1000\" x1=\"0\" y2=\"1000\" x2=\"0\" stroke=\"black\" stroke-width=\"0.1\" stroke-opacity=\"0.4\" />\n");
}

void end_drawing() {
	printf("</svg>\n");
}
