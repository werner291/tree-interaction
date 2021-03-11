struct ConicalFrustrum<N> {
    half_height: N,
    radius_1: N,
    radius_2: N,
}

struct ConicalFrustrumDiscretizationParameters {
    n_segments: usize,
    make_caps: bool,
}
//
// impl ToTriMesh<N> for ConicalFrustrum<N> {
//     type DiscretizationParameter = ConicalFrustrumDiscretizationParameters;
//
//     fn to_trimesh(&self, i: Self::DiscretizationParameter) -> TriMesh<N> {
//
//
//
//     }
// }
