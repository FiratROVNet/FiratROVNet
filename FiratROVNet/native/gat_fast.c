#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <math.h>
#include <stdlib.h>

#include <Python.h>
#include <numpy/arrayobject.h>

static float clip01(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static PyObject *gat_graph_hesapla(PyObject *self, PyObject *args) {
    PyObject *positions_obj = NULL;
    PyObject *velocities_obj = NULL;
    PyObject *batteries_obj = NULL;
    PyObject *roles_obj = NULL;
    PyObject *engels_obj = NULL;
    double leader_limit = 0.0;
    double disconnect_limit = 0.0;
    double obstacle_limit = 0.0;
    double collision_limit = 0.0;
    int max_komsu = 0;

    if (!PyArg_ParseTuple(
            args,
            "OOOOOddddi",
            &positions_obj,
            &velocities_obj,
            &batteries_obj,
            &roles_obj,
            &engels_obj,
            &leader_limit,
            &disconnect_limit,
            &obstacle_limit,
            &collision_limit,
            &max_komsu)) {
        return NULL;
    }

    PyArrayObject *positions = (PyArrayObject *)PyArray_FROM_OTF(positions_obj, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);
    PyArrayObject *velocities = (PyArrayObject *)PyArray_FROM_OTF(velocities_obj, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);
    PyArrayObject *batteries = (PyArrayObject *)PyArray_FROM_OTF(batteries_obj, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);
    PyArrayObject *roles = (PyArrayObject *)PyArray_FROM_OTF(roles_obj, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);
    PyArrayObject *engels = (PyArrayObject *)PyArray_FROM_OTF(engels_obj, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);

    if (!positions || !velocities || !batteries || !roles || !engels) {
        Py_XDECREF(positions);
        Py_XDECREF(velocities);
        Py_XDECREF(batteries);
        Py_XDECREF(roles);
        Py_XDECREF(engels);
        return NULL;
    }

    int ndim_ok = PyArray_NDIM(positions) == 2 && PyArray_DIM(positions, 1) == 3 &&
                  PyArray_NDIM(velocities) == 2 && PyArray_DIM(velocities, 1) == 3 &&
                  PyArray_NDIM(batteries) == 1 &&
                  PyArray_NDIM(roles) == 1 &&
                  PyArray_NDIM(engels) == 2 && PyArray_DIM(engels, 1) == 3;
    if (!ndim_ok) {
        PyErr_SetString(PyExc_ValueError, "invalid array shapes for gat_graph_hesapla");
        Py_DECREF(positions);
        Py_DECREF(velocities);
        Py_DECREF(batteries);
        Py_DECREF(roles);
        Py_DECREF(engels);
        return NULL;
    }

    npy_intp n_npy = PyArray_DIM(positions, 0);
    npy_intp e_npy = PyArray_DIM(engels, 0);
    if (PyArray_DIM(velocities, 0) != n_npy || PyArray_DIM(batteries, 0) != n_npy || PyArray_DIM(roles, 0) != n_npy) {
        PyErr_SetString(PyExc_ValueError, "ROV arrays must have matching lengths");
        Py_DECREF(positions);
        Py_DECREF(velocities);
        Py_DECREF(batteries);
        Py_DECREF(roles);
        Py_DECREF(engels);
        return NULL;
    }

    npy_intp x_dims[2] = {n_npy, 9};
    PyArrayObject *x_out = (PyArrayObject *)PyArray_SimpleNew(2, x_dims, NPY_FLOAT32);
    if (!x_out) {
        Py_DECREF(positions);
        Py_DECREF(velocities);
        Py_DECREF(batteries);
        Py_DECREF(roles);
        Py_DECREF(engels);
        return NULL;
    }

    float *pos = (float *)PyArray_DATA(positions);
    float *vel = (float *)PyArray_DATA(velocities);
    float *bat = (float *)PyArray_DATA(batteries);
    float *role = (float *)PyArray_DATA(roles);
    float *eng = (float *)PyArray_DATA(engels);
    float *x = (float *)PyArray_DATA(x_out);

    npy_intp n = n_npy;
    npy_intp eng_count = e_npy;
    float *dist = (float *)malloc((size_t)n * (size_t)n * sizeof(float));
    float *min_dist = (float *)malloc((size_t)n * sizeof(float));
    float *leader_dist = (float *)malloc((size_t)n * sizeof(float));
    float *codes = (float *)calloc((size_t)n, sizeof(float));

    if (!dist || !min_dist || !leader_dist || !codes) {
        free(dist);
        free(min_dist);
        free(leader_dist);
        free(codes);
        PyErr_NoMemory();
        Py_DECREF(positions);
        Py_DECREF(velocities);
        Py_DECREF(batteries);
        Py_DECREF(roles);
        Py_DECREF(engels);
        Py_DECREF(x_out);
        return NULL;
    }

    const float inf = INFINITY;
    for (npy_intp i = 0; i < n; i++) {
        min_dist[i] = inf;
        for (npy_intp j = 0; j < n; j++) {
            if (i == j) {
                dist[i * n + j] = inf;
                continue;
            }
            float dx = pos[i * 3] - pos[j * 3];
            float dy = pos[i * 3 + 1] - pos[j * 3 + 1];
            float dz = pos[i * 3 + 2] - pos[j * 3 + 2];
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            dist[i * n + j] = d;
            if (d < min_dist[i]) {
                min_dist[i] = d;
            }
        }
    }

    npy_intp leader_idx = 0;
    for (npy_intp i = 0; i < n; i++) {
        if (role[i] > 0.5f) {
            leader_idx = i;
            break;
        }
    }

    for (npy_intp i = 0; i < n; i++) {
        leader_dist[i] = (n > 1) ? dist[i * n + leader_idx] : 0.0f;
    }
    if (leader_idx >= 0 && leader_idx < n) {
        leader_dist[leader_idx] = 0.0f;
    }

    for (npy_intp i = 0; i < n; i++) {
        if (role[i] < 0.5f && leader_dist[i] > (float)leader_limit) {
            codes[i] = 5.0f;
        }
        if (min_dist[i] > (float)disconnect_limit) {
            codes[i] = 3.0f;
        }
    }

    if (eng_count > 0) {
        for (npy_intp i = 0; i < n; i++) {
            float min_engel = inf;
            for (npy_intp j = 0; j < eng_count; j++) {
                float dx = pos[i * 3] - eng[j * 3];
                float dy = pos[i * 3 + 1] - eng[j * 3 + 1];
                float dz = pos[i * 3 + 2] - eng[j * 3 + 2];
                float d = sqrtf(dx * dx + dy * dy + dz * dz);
                if (d < min_engel) {
                    min_engel = d;
                }
            }
            if (min_engel - 6.0f < (float)obstacle_limit) {
                codes[i] = 1.0f;
            }
        }
    }

    if (n > 1) {
        for (npy_intp i = 0; i < n; i++) {
            for (npy_intp j = 0; j < n; j++) {
                if (i != j && dist[i * n + j] < (float)collision_limit) {
                    codes[i] = 2.0f;
                    break;
                }
            }
        }
    }

    for (npy_intp i = 0; i < n; i++) {
        npy_intp base = i * 9;
        x[base] = codes[i] / 5.0f;
        x[base + 1] = bat[i];
        x[base + 2] = 0.9f;
        x[base + 3] = clip01(fabsf(pos[i * 3 + 1]) / 100.0f);
        x[base + 4] = vel[i * 3];
        x[base + 5] = vel[i * 3 + 2];
        x[base + 6] = role[i];
        x[base + 7] = clip01(min_dist[i] / 100.0f);
        x[base + 8] = clip01(leader_dist[i] / 100.0f);
    }

    npy_intp max_edges = 0;
    if (n > 1) {
        if (max_komsu > 0 && n - 1 > max_komsu) {
            max_edges = n * (npy_intp)max_komsu;
        } else {
            max_edges = n * (n - 1);
        }
    }

    npy_intp edge_dims[2] = {2, max_edges};
    PyArrayObject *edge_out = (PyArrayObject *)PyArray_SimpleNew(2, edge_dims, NPY_INT64);
    if (!edge_out) {
        free(dist);
        free(min_dist);
        free(leader_dist);
        free(codes);
        Py_DECREF(positions);
        Py_DECREF(velocities);
        Py_DECREF(batteries);
        Py_DECREF(roles);
        Py_DECREF(engels);
        Py_DECREF(x_out);
        return NULL;
    }
    npy_int64 *edge = (npy_int64 *)PyArray_DATA(edge_out);
    npy_intp edge_count = 0;

    if (n > 1) {
        if (max_komsu > 0 && n - 1 > max_komsu) {
            float *best_d = (float *)malloc((size_t)max_komsu * sizeof(float));
            npy_intp *best_j = (npy_intp *)malloc((size_t)max_komsu * sizeof(npy_intp));
            if (!best_d || !best_j) {
                free(best_d);
                free(best_j);
                free(dist);
                free(min_dist);
                free(leader_dist);
                free(codes);
                PyErr_NoMemory();
                Py_DECREF(positions);
                Py_DECREF(velocities);
                Py_DECREF(batteries);
                Py_DECREF(roles);
                Py_DECREF(engels);
                Py_DECREF(x_out);
                Py_DECREF(edge_out);
                return NULL;
            }

            for (npy_intp i = 0; i < n; i++) {
                int count = 0;
                int farthest = 0;
                for (npy_intp j = 0; j < n; j++) {
                    if (i == j) {
                        continue;
                    }
                    float d = dist[i * n + j];
                    if (d >= (float)disconnect_limit) {
                        continue;
                    }
                    if (count < max_komsu) {
                        best_d[count] = d;
                        best_j[count] = j;
                        if (best_d[count] > best_d[farthest]) {
                            farthest = count;
                        }
                        count++;
                    } else if (d < best_d[farthest]) {
                        best_d[farthest] = d;
                        best_j[farthest] = j;
                        farthest = 0;
                        for (int k = 1; k < max_komsu; k++) {
                            if (best_d[k] > best_d[farthest]) {
                                farthest = k;
                            }
                        }
                    }
                }
                for (int k = 0; k < count; k++) {
                    edge[edge_count] = (npy_int64)i;
                    edge[max_edges + edge_count] = (npy_int64)best_j[k];
                    edge_count++;
                }
            }
            free(best_d);
            free(best_j);
        } else {
            for (npy_intp i = 0; i < n; i++) {
                for (npy_intp j = 0; j < n; j++) {
                    if (i != j && dist[i * n + j] < (float)disconnect_limit) {
                        edge[edge_count] = (npy_int64)i;
                        edge[max_edges + edge_count] = (npy_int64)j;
                        edge_count++;
                    }
                }
            }
        }
    }

    PyObject *edge_trimmed = NULL;
    if (edge_count == max_edges) {
        edge_trimmed = (PyObject *)edge_out;
        Py_INCREF(edge_trimmed);
    } else {
        npy_intp trimmed_dims[2] = {2, edge_count};
        PyArrayObject *trimmed = (PyArrayObject *)PyArray_SimpleNew(2, trimmed_dims, NPY_INT64);
        if (!trimmed) {
            free(dist);
            free(min_dist);
            free(leader_dist);
            free(codes);
            Py_DECREF(positions);
            Py_DECREF(velocities);
            Py_DECREF(batteries);
            Py_DECREF(roles);
            Py_DECREF(engels);
            Py_DECREF(x_out);
            Py_DECREF(edge_out);
            return NULL;
        }
        npy_int64 *trim_data = (npy_int64 *)PyArray_DATA(trimmed);
        for (npy_intp i = 0; i < edge_count; i++) {
            trim_data[i] = edge[i];
            trim_data[edge_count + i] = edge[max_edges + i];
        }
        edge_trimmed = (PyObject *)trimmed;
    }

    free(dist);
    free(min_dist);
    free(leader_dist);
    free(codes);
    Py_DECREF(positions);
    Py_DECREF(velocities);
    Py_DECREF(batteries);
    Py_DECREF(roles);
    Py_DECREF(engels);
    Py_DECREF(edge_out);

    PyObject *result = Py_BuildValue("NN", (PyObject *)x_out, edge_trimmed);
    return result;
}

static PyObject *rov_kacinma_hesapla(PyObject *self, PyObject *args) {
    PyObject *self_pos_obj = NULL;
    PyObject *other_pos_obj = NULL;
    PyObject *other_ids_obj = NULL;
    double menzil = 0.0;

    if (!PyArg_ParseTuple(args, "OOOd", &self_pos_obj, &other_pos_obj, &other_ids_obj, &menzil)) {
        return NULL;
    }

    PyArrayObject *self_pos = (PyArrayObject *)PyArray_FROM_OTF(self_pos_obj, NPY_FLOAT64, NPY_ARRAY_IN_ARRAY);
    PyArrayObject *other_pos = (PyArrayObject *)PyArray_FROM_OTF(other_pos_obj, NPY_FLOAT64, NPY_ARRAY_IN_ARRAY);
    PyArrayObject *other_ids = (PyArrayObject *)PyArray_FROM_OTF(other_ids_obj, NPY_INT64, NPY_ARRAY_IN_ARRAY);

    if (!self_pos || !other_pos || !other_ids) {
        Py_XDECREF(self_pos);
        Py_XDECREF(other_pos);
        Py_XDECREF(other_ids);
        return NULL;
    }

    if (PyArray_NDIM(self_pos) != 1 || PyArray_DIM(self_pos, 0) < 3 ||
        PyArray_NDIM(other_pos) != 2 || PyArray_DIM(other_pos, 1) < 3 ||
        PyArray_NDIM(other_ids) != 1 || PyArray_DIM(other_ids, 0) != PyArray_DIM(other_pos, 0)) {
        PyErr_SetString(PyExc_ValueError, "invalid array shapes for rov_kacinma_hesapla");
        Py_DECREF(self_pos);
        Py_DECREF(other_pos);
        Py_DECREF(other_ids);
        return NULL;
    }

    npy_intp n = PyArray_DIM(other_pos, 0);
    npy_intp out_dims[1] = {n};
    PyArrayObject *ids_out = (PyArrayObject *)PyArray_SimpleNew(1, out_dims, NPY_INT64);
    PyArrayObject *dist_out = (PyArrayObject *)PyArray_SimpleNew(1, out_dims, NPY_FLOAT64);
    npy_intp vec_dims[2] = {n, 3};
    PyArrayObject *coord_out = (PyArrayObject *)PyArray_SimpleNew(2, vec_dims, NPY_FLOAT64);
    PyArrayObject *unit_out = (PyArrayObject *)PyArray_SimpleNew(2, vec_dims, NPY_FLOAT64);

    if (!ids_out || !dist_out || !coord_out || !unit_out) {
        Py_XDECREF(ids_out);
        Py_XDECREF(dist_out);
        Py_XDECREF(coord_out);
        Py_XDECREF(unit_out);
        Py_DECREF(self_pos);
        Py_DECREF(other_pos);
        Py_DECREF(other_ids);
        return NULL;
    }

    double *sp = (double *)PyArray_DATA(self_pos);
    double *op = (double *)PyArray_DATA(other_pos);
    npy_int64 *oid = (npy_int64 *)PyArray_DATA(other_ids);
    npy_int64 *ids = (npy_int64 *)PyArray_DATA(ids_out);
    double *distances = (double *)PyArray_DATA(dist_out);
    double *coords = (double *)PyArray_DATA(coord_out);
    double *units = (double *)PyArray_DATA(unit_out);

    npy_intp count = 0;
    for (npy_intp i = 0; i < n; i++) {
        double dx = op[i * 3] - sp[0];
        double dy = op[i * 3 + 1] - sp[1];
        double dz = op[i * 3 + 2] - sp[2];
        double d = sqrt(dx * dx + dy * dy + dz * dz);
        if (d <= menzil && d > 1e-9) {
            ids[count] = oid[i];
            distances[count] = d;
            coords[count * 3] = op[i * 3];
            coords[count * 3 + 1] = op[i * 3 + 1];
            coords[count * 3 + 2] = op[i * 3 + 2];
            units[count * 3] = -dx / d;
            units[count * 3 + 1] = -dy / d;
            units[count * 3 + 2] = -dz / d;
            count++;
        }
    }

    npy_intp trim_1d[1] = {count};
    npy_intp trim_2d[2] = {count, 3};
    PyArrayObject *ids_trim = (PyArrayObject *)PyArray_SimpleNew(1, trim_1d, NPY_INT64);
    PyArrayObject *dist_trim = (PyArrayObject *)PyArray_SimpleNew(1, trim_1d, NPY_FLOAT64);
    PyArrayObject *coord_trim = (PyArrayObject *)PyArray_SimpleNew(2, trim_2d, NPY_FLOAT64);
    PyArrayObject *unit_trim = (PyArrayObject *)PyArray_SimpleNew(2, trim_2d, NPY_FLOAT64);
    if (!ids_trim || !dist_trim || !coord_trim || !unit_trim) {
        Py_XDECREF(ids_trim);
        Py_XDECREF(dist_trim);
        Py_XDECREF(coord_trim);
        Py_XDECREF(unit_trim);
        Py_DECREF(self_pos);
        Py_DECREF(other_pos);
        Py_DECREF(other_ids);
        Py_DECREF(ids_out);
        Py_DECREF(dist_out);
        Py_DECREF(coord_out);
        Py_DECREF(unit_out);
        return NULL;
    }
    npy_int64 *ids_trim_data = (npy_int64 *)PyArray_DATA(ids_trim);
    double *dist_trim_data = (double *)PyArray_DATA(dist_trim);
    double *coord_trim_data = (double *)PyArray_DATA(coord_trim);
    double *unit_trim_data = (double *)PyArray_DATA(unit_trim);
    for (npy_intp i = 0; i < count; i++) {
        ids_trim_data[i] = ids[i];
        dist_trim_data[i] = distances[i];
        for (int j = 0; j < 3; j++) {
            coord_trim_data[i * 3 + j] = coords[i * 3 + j];
            unit_trim_data[i * 3 + j] = units[i * 3 + j];
        }
    }

    Py_DECREF(self_pos);
    Py_DECREF(other_pos);
    Py_DECREF(other_ids);
    Py_DECREF(ids_out);
    Py_DECREF(dist_out);
    Py_DECREF(coord_out);
    Py_DECREF(unit_out);

    return Py_BuildValue("NNNN", (PyObject *)ids_trim, (PyObject *)coord_trim, (PyObject *)unit_trim, (PyObject *)dist_trim);
}

static PyObject *statik_engeller_hesapla(PyObject *self, PyObject *args) {
    double rx = 0.0, ry = 0.0, rz = 0.0, menzil = 0.0;
    PyObject *islands_obj = NULL;

    if (!PyArg_ParseTuple(args, "ddddO", &rx, &ry, &rz, &menzil, &islands_obj)) {
        return NULL;
    }

    PyArrayObject *islands = (PyArrayObject *)PyArray_FROM_OTF(islands_obj, NPY_FLOAT64, NPY_ARRAY_IN_ARRAY);
    if (!islands) {
        return NULL;
    }
    if (PyArray_NDIM(islands) != 2 || PyArray_DIM(islands, 1) < 3) {
        PyErr_SetString(PyExc_ValueError, "invalid island array shape");
        Py_DECREF(islands);
        return NULL;
    }

    npy_intp n = PyArray_DIM(islands, 0);
    npy_intp out_dims[1] = {n};
    npy_intp coord_dims[2] = {n, 3};
    PyArrayObject *dist_out = (PyArrayObject *)PyArray_SimpleNew(1, out_dims, NPY_FLOAT64);
    PyArrayObject *radius_out = (PyArrayObject *)PyArray_SimpleNew(1, out_dims, NPY_FLOAT64);
    PyArrayObject *coord_out = (PyArrayObject *)PyArray_SimpleNew(2, coord_dims, NPY_FLOAT64);

    if (!dist_out || !radius_out || !coord_out) {
        Py_XDECREF(dist_out);
        Py_XDECREF(radius_out);
        Py_XDECREF(coord_out);
        Py_DECREF(islands);
        return NULL;
    }

    double *arr = (double *)PyArray_DATA(islands);
    double *distances = (double *)PyArray_DATA(dist_out);
    double *radii = (double *)PyArray_DATA(radius_out);
    double *coords = (double *)PyArray_DATA(coord_out);
    npy_intp count = 0;

    for (npy_intp i = 0; i < n; i++) {
        double ix = arr[i * 3];
        double iz = arr[i * 3 + 1];
        double ir = arr[i * 3 + 2];
        double dx = ix - rx;
        double dz = iz - rz;
        double d_center = sqrt(dx * dx + dz * dz);
        if (d_center < 1e-6) {
            continue;
        }
        double surface = d_center - ir;
        if (surface < 0.0) {
            surface = 0.0;
        }
        if (surface >= menzil) {
            continue;
        }
        double scale = (d_center - ir) / d_center;
        coords[count * 3] = rx + dx * scale;
        coords[count * 3 + 1] = rz + dz * scale;
        coords[count * 3 + 2] = ry;
        distances[count] = surface;
        radii[count] = ir;
        count++;
    }

    npy_intp trim_1d[1] = {count};
    npy_intp trim_2d[2] = {count, 3};
    PyArrayObject *coord_trim = (PyArrayObject *)PyArray_SimpleNew(2, trim_2d, NPY_FLOAT64);
    PyArrayObject *dist_trim = (PyArrayObject *)PyArray_SimpleNew(1, trim_1d, NPY_FLOAT64);
    PyArrayObject *radius_trim = (PyArrayObject *)PyArray_SimpleNew(1, trim_1d, NPY_FLOAT64);
    if (!coord_trim || !dist_trim || !radius_trim) {
        Py_XDECREF(coord_trim);
        Py_XDECREF(dist_trim);
        Py_XDECREF(radius_trim);
        Py_DECREF(islands);
        Py_DECREF(dist_out);
        Py_DECREF(radius_out);
        Py_DECREF(coord_out);
        return NULL;
    }
    double *coord_trim_data = (double *)PyArray_DATA(coord_trim);
    double *dist_trim_data = (double *)PyArray_DATA(dist_trim);
    double *radius_trim_data = (double *)PyArray_DATA(radius_trim);
    for (npy_intp i = 0; i < count; i++) {
        dist_trim_data[i] = distances[i];
        radius_trim_data[i] = radii[i];
        for (int j = 0; j < 3; j++) {
            coord_trim_data[i * 3 + j] = coords[i * 3 + j];
        }
    }

    Py_DECREF(islands);
    Py_DECREF(dist_out);
    Py_DECREF(radius_out);
    Py_DECREF(coord_out);

    return Py_BuildValue("NNN", (PyObject *)coord_trim, (PyObject *)dist_trim, (PyObject *)radius_trim);
}

static PyMethodDef GatFastMethods[] = {
    {"gat_graph_hesapla", gat_graph_hesapla, METH_VARARGS, "Build GAT x and edge_index arrays quickly."},
    {"rov_kacinma_hesapla", rov_kacinma_hesapla, METH_VARARGS, "Compute nearby ROV avoidance vectors."},
    {"statik_engeller_hesapla", statik_engeller_hesapla, METH_VARARGS, "Compute nearby static island obstacle points."},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef gat_fast_module = {
    PyModuleDef_HEAD_INIT,
    "gat_fast",
    "Native GAT graph helpers.",
    -1,
    GatFastMethods
};

PyMODINIT_FUNC PyInit_gat_fast(void) {
    import_array();
    return PyModule_Create(&gat_fast_module);
}
