// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Terrain Painter/Projector"
{
	Properties
	{
		_MainTex ("Brush", 2D) = "" { }
		_Color ("Color", Color) = (0.0, 0.34, 1.0, 1.0)
	}

	SubShader
	{
		Pass
		{ 
			Tags
			{
				"RenderType" = "Transparent"
				"Queue" = "Transparent+100"
			}

			Fog{ Mode Off }

			ZWrite Off
			Blend SrcAlpha One

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag 

			uniform sampler2D _MainTex;		// Alpha texture
			uniform float4 _Color;			// Color
			uniform float4x4 unity_Projector;	// Projector

			struct vertexInput
			{ // in
				float4 vertex : POSITION;
				float3 normal : NORMAL;
			};

			struct vertexOutput
			{ // out
				float4 pos : SV_POSITION;
				float4 posProj : TEXCOORD0;
			};

			vertexOutput vert (vertexInput input)
			{ // vertex
				vertexOutput output;

				output.posProj = mul(unity_Projector, input.vertex);
				output.pos = UnityObjectToClipPos(input.vertex);
				return output;
			}

			float4 frag(vertexOutput input) : COLOR
			{ // fragment
				float2 uv = input.posProj.xy / input.posProj.w;
				if (uv.x <= 0 || uv.x >= 1 ||
					uv.y <= 0 || uv.y >= 1)
					return 0;
				return _Color * tex2D(_MainTex, uv).a;
			}

			ENDCG
		}
	}
}
