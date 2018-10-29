Shader "Custom/TerrainVertexMasked" {
	Properties {
		_Color ("Zero Color", Color) = (0,0,0,1)
		[NoScaleOffset] _MainTex ("Zero Diffuse", 2D) = "white" {}
		[NoScaleOffset] [Normal] _Normal ("Zero Normal", 2D) = "bump" {}
		_Specular ("Zero Specular", Color) = (0.1,0.1,0.1,0.25)
		_Tiling ("Zero Tiling", Float) = 1.0

		_ColorRed ("Red Color", Color) = (1,1,1,1)
		[NoScaleOffset] _MainTexRed ("Red Diffuse", 2D) = "white" {}
		[NoScaleOffset] [Normal] _NormalRed ("Red Normal", 2D) = "bump" {}
		_SpecularRed ("Red Specular", Color) = (0.1,0.1,0.1,0.25)
		_TilingRed ("Red Tiling", Float) = 1.0

		_ColorGreen ("Green Color", Color) = (1,1,1,1)
		[NoScaleOffset] _MainTexGreen ("Green Diffuse", 2D) = "white" {}
		[NoScaleOffset] [Normal] _NormalGreen ("Green Normal", 2D) = "bump" {}
		_SpecularGreen ("Green Specular", Color) = (0.1,0.1,0.1,0.25)
		_TilingGreen ("Green Tiling", Float) = 1.0
	}
	SubShader {
		Tags { "RenderType"="Opaque" }
		LOD 200

		CGPROGRAM
		// Physically based Standard lighting model, and enable shadows on all light types
		#pragma surface surf StandardSpecular fullforwardshadows

		// Use shader model 3.0 target, to get nicer looking lighting
		#pragma target 3.0

		sampler2D _MainTex;
		sampler2D _MainTexRed;
		sampler2D _MainTexGreen;

		sampler2D _Normal;
		sampler2D _NormalRed;
		sampler2D _NormalGreen;

		struct Input {
			float2 uv_MainTex;
			float4 color : COLOR;
		};

		fixed4 _Color;
		fixed4 _Specular;
		fixed _Tiling;
		fixed4 _ColorRed;
		fixed4 _SpecularRed;
		fixed _TilingRed;
		fixed4 _ColorGreen;
		fixed4 _SpecularGreen;
		fixed _TilingGreen;

		// Add instancing support for this shader. You need to check 'Enable Instancing' on materials that use the shader.
		// See https://docs.unity3d.com/Manual/GPUInstancing.html for more information about instancing.
		// #pragma instancing_options assumeuniformscaling
		UNITY_INSTANCING_BUFFER_START(Props)
			// put more per-instance properties here
		UNITY_INSTANCING_BUFFER_END(Props)

		void surf (Input v, inout SurfaceOutputStandardSpecular o) {
			
			fixed4 zeroDiffuse = tex2D (_MainTex, v.uv_MainTex * _Tiling) * _Color;
			fixed4 redDiffuse = tex2D (_MainTexRed, v.uv_MainTex * _TilingRed) * _ColorRed;
			fixed4 greenDiffuse = tex2D (_MainTexGreen, v.uv_MainTex * _TilingGreen) * _ColorGreen;

			float4 zeroNormal = tex2D (_Normal, v.uv_MainTex * _Tiling);
			float4 redNormal = tex2D (_NormalRed, v.uv_MainTex * _TilingRed);
			float4 greenNormal = tex2D (_NormalGreen, v.uv_MainTex * _TilingGreen);

			zeroNormal = UnpackNormal(zeroNormal).rgbb;
			redNormal = UnpackNormal(redNormal).rgbb;
			greenNormal = UnpackNormal(greenNormal).rgbb;

			fixed greenLerp = smoothstep((1 - greenDiffuse.a) * 0.9999, 1, v.color.g);// / (v.color.g + 0.0001);

			fixed4 finalDiffuse = lerp(lerp(zeroDiffuse, redDiffuse, v.color.r), greenDiffuse, greenLerp);

			fixed4 finalSpecular = lerp(lerp(_Specular, _SpecularRed, v.color.r), _SpecularGreen, greenLerp);

			half4 finalNormal = lerp(lerp(zeroNormal, redNormal, v.color.r), greenNormal, greenLerp);

			o.Albedo = finalDiffuse.rgb;
			o.Specular = finalSpecular.rgb;
			o.Normal = finalNormal;
			o.Smoothness = finalSpecular.a;

			o.Occlusion = v.color.b;

		}
		ENDCG
	}
	FallBack "Diffuse"
}
