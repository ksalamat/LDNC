## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    obj = bld.create_ns3_program('LDNC', ['core', 'mobility', 'wifi', 'applications', 'point-to-point',
                                                    'internet', 'csma', 'olsr', 'netanim'])
    obj.source = ['Utils.cc', 'GaloisField.cpp', 'LDNC+BF+Aging.cc']
    obj.env.append_value("LINKFLAGS",["-L/usr/local/lib"])
    obj.env.append_value("LIB", ["glpk"])
